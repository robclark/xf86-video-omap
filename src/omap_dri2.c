/* -*- mode: C; c-file-style: "k&r"; tab-width 4; indent-tabs-mode: t; -*- */

/*
 * Copyright © 2011 Texas Instruments, Inc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Rob Clark <rob@ti.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "omap_driver.h"
#include "omap_exa.h"

#include "xf86drmMode.h"
#include "dri2.h"

/* any point to support earlier? */
#if DRI2INFOREC_VERSION < 7
#	error "Requires newer DRI2"
#endif


typedef struct {
	DRI2BufferRec base;

	/**
	 * Pixmap that is backing the buffer
	 *
	 * NOTE: don't track the pixmap ptr for the front buffer if it is
	 * a window.. this could get reallocated from beneath us, so we should
	 * always use draw2pix to be sure to have the correct one
	 */
	PixmapPtr pPixmap;

	/**
	 * The drm_framebuffer id for the buffer.. or 0 if the buffer cannot be
	 * directly scanned out by the hw (in which case we need to fall back to
	 * blitting)
	 */
	uint32_t fb_id;

	/**
	 * For multi-planar formats, extra buffer info..
	 */
	int extraCount;
	PixmapPtr *extraPix;
	unsigned int *extraNames;
	unsigned int *extraPitches;

} OMAPDRI2BufferRec, *OMAPDRI2BufferPtr;

#define OMAPBUF(p)	((OMAPDRI2BufferPtr)(p))
#define DRIBUF(p)	((DRI2BufferPtr)(&(p)->base))


static inline DrawablePtr
dri2draw(DrawablePtr pDraw, DRI2BufferPtr buf)
{
	if (buf->attachment == DRI2BufferFrontLeft) {
		return pDraw;
	} else {
		return &(OMAPBUF(buf)->pPixmap->drawable);
	}
}

static inline Bool
canexchange(DrawablePtr pDraw, DRI2BufferPtr a, DRI2BufferPtr b)
{
	DrawablePtr da = dri2draw(pDraw, a);
	DrawablePtr db = dri2draw(pDraw, b);

	return DRI2CanFlip(pDraw) &&
			(da->width == db->width) &&
			(da->height == db->height) &&
			(da->depth == db->depth);
}

static Bool
canflip(DrawablePtr pDraw)
{
	return (pDraw->type == DRAWABLE_WINDOW) &&
			DRI2CanFlip(pDraw);
}

static inline Bool
exchangebufs(DrawablePtr pDraw, DRI2BufferPtr a, DRI2BufferPtr b)
{
	OMAPPixmapExchange(draw2pix(dri2draw(pDraw, a)),
			draw2pix(dri2draw(pDraw, b)));
	exchange(a->name, b->name);
	return TRUE;
}

static PixmapPtr
createpix(DrawablePtr pDraw)
{
	ScreenPtr pScreen = pDraw->pScreen;
	int flags = canflip(pDraw) ? OMAP_CREATE_PIXMAP_SCANOUT : 0;
	return pScreen->CreatePixmap(pScreen,
			pDraw->width, pDraw->height, pDraw->depth, flags);
}

static OMAPDRI2BufferPtr
createbuf(DrawablePtr pDraw, PixmapPtr pPixmap, int attachment, int format)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPDRI2BufferPtr buf = calloc(1, sizeof(*buf));
	int ret;

	if (!buf) {
		return NULL;
	}

	DRIBUF(buf)->attachment = attachment;
	DRIBUF(buf)->pitch = exaGetPixmapPitch(pPixmap);
	DRIBUF(buf)->cpp = pPixmap->drawable.bitsPerPixel / 8;
	DRIBUF(buf)->format = format;

	ret = omap_bo_get_name(OMAPPixmapBo(pPixmap), &DRIBUF(buf)->name);
	if (ret) {
		ERROR_MSG("could not get buffer name: %d", ret);
		return NULL;
	}

	if (attachment != DRI2BufferFrontLeft) {
		buf->pPixmap = pPixmap;
	}

	return buf;
}

/**
 * Create Buffer.
 *
 * Note that 'format' is used from the client side to specify the DRI buffer
 * format, which could differ from the drawable format.  For example, the
 * drawable could be 32b RGB, but the DRI buffer some YUV format (video) or
 * perhaps lower bit depth RGB (GL).  The color conversion is handled when
 * blitting to front buffer, and page-flipping (overlay or flipchain) can
 * only be used if the display supports.
 */
static DRI2BufferPtr
OMAPDRI2CreateBuffer(DrawablePtr pDraw, unsigned int attachment,
		unsigned int format)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	OMAPDRI2BufferPtr buf;
	PixmapPtr pPixmap;

	DEBUG_MSG("pDraw=%p, attachment=%d, format=%08x",
			pDraw, attachment, format);

	if (attachment == DRI2BufferFrontLeft) {
		pPixmap = draw2pix(pDraw);

		/* to do flipping, we need a scanout capable (physically contiguous)
		 * buffer.. this bit of gymnastics ensures that.
		 *
		 * TODO we may need to re-allocate and switch back to non-scanout
		 * buffer when client disconnects from drawable..
		 */
		if (canflip(pDraw) && !(has_dmm(pOMAP) ||
				(OMAPPixmapBo(pPixmap) == pOMAP->scanout))) {
			/* need to re-allocate pixmap to get a scanout capable buffer */
			PixmapPtr pNewPix = createpix(pDraw);

			// TODO copy contents..

			OMAPPixmapExchange(pPixmap, pNewPix);

			pScreen->DestroyPixmap(pNewPix);
		}
	} else {
		pPixmap = createpix(pDraw);
	}

	buf = createbuf(pDraw, pPixmap, attachment, format);

	/* Q: how to know across OMAP generations what formats that the display
	 * can support directly?
	 * A: attempt to create a drm_framebuffer, and if that fails then the
	 * hw must not support.. if buf->fb_id==0 then fall back to blitting
	 */
	if (canflip(pDraw)) {
		int ret = drmModeAddFB(pOMAP->drmFD, pDraw->width, pDraw->height,
				pDraw->depth, pDraw->bitsPerPixel, DRIBUF(buf)->pitch,
				omap_bo_handle(OMAPPixmapBo(pPixmap)), &buf->fb_id);
		if (ret) {
			/* to-bad, so-sad, we can't flip */
			WARNING_MSG("could not create fb: %d", ret);
			buf->fb_id = 0;
		}
	}

	return DRIBUF(buf);
}

static DRI2BufferPtr
OMAPDRI2CreateBufferVid(DrawablePtr pDraw, unsigned int attachment,
		unsigned int format, unsigned int width, unsigned int height)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPDRI2BufferPtr buf;
	PixmapPtr pPixmap;
	int bpp, extraCount = 0, ew = width, eh = height;

	DEBUG_MSG("pDraw=%p, attachment=%d, format=%08x",
			pDraw, attachment, format);

	switch(format) {
	case fourcc_code('I','4','2','0'):
	case fourcc_code('Y','V','1','2'):
		extraCount++;
		ew /= 2;
		/* fallthru */
	case fourcc_code('N','V','1','2'):
		extraCount++;
		eh /= 2;
		bpp = 8;
		break;
	case fourcc_code('U','Y','V','Y'):
	case fourcc_code('Y','U','Y','2'):
		bpp = 16;
		break;
	default:
		return NULL;
	}

	/* main buffer, luma buffer in case of multi-planar.. */
	pPixmap = pScreen->CreatePixmap(pScreen, width, height, bpp, 0);

	buf = createbuf(pDraw, pPixmap, attachment, format);

	if (extraCount > 0) {
		int i, ret;

		buf->extraPix = malloc(extraCount * sizeof(buf->extraPix[0]));
		buf->extraNames = malloc(extraCount * sizeof(buf->extraNames[0]));
		buf->extraPitches = malloc(extraCount * sizeof(buf->extraPitches[0]));
		buf->extraCount = extraCount;

		for (i = 0; i < extraCount; i++) {
			PixmapPtr p = pScreen->CreatePixmap(pScreen, ew, eh, bpp, 0);

			buf->extraPix[i] = p;
			buf->extraPitches[i] = exaGetPixmapPitch(p);

			ret = omap_bo_get_name(OMAPPixmapBo(p), &buf->extraNames[i]);
			if (ret) {
				ERROR_MSG("could not get buffer name: %d", ret);
				/* TODO: cleanup.. */
				return NULL;
			}
		}
	}

	return DRIBUF(buf);
}

/**
 * An ugly approach to avoid changing DRI2BufferPtr and cause ABI breakage
 * between driver and xserver.  This only needs to be implemented by drivers
 * supporting planar formats with one buffer per plane.
 */
static unsigned int
OMAPDRI2GetExtraBufferNames(DrawablePtr pDraw, DRI2BufferPtr buffer,
		unsigned int **names, unsigned int **pitches)
{
	OMAPDRI2BufferPtr buf = OMAPBUF(buffer);
	*names = buf->extraNames;
	*pitches = buf->extraPitches;
	return buf->extraCount;
}

/**
 * Destroy Buffer
 *
 * TODO: depending on how flipping ends up working, we may need a refcnt or
 * something like this to defer destroying a buffer that is currently being
 * scanned out..
 */
static void
OMAPDRI2DestroyBuffer(DrawablePtr pDraw, DRI2BufferPtr buffer)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	OMAPDRI2BufferPtr buf = OMAPBUF(buffer);
	int i;

	DEBUG_MSG("pDraw=%p, buffer=%p", pDraw, buffer);

	if (buf->fb_id) {
		drmModeRmFB(pOMAP->drmFD, buf->fb_id);
	}

	if (DRIBUF(buf)->attachment == DRI2BufferFrontLeft) {
		// XXX don't destroy.. I think..
	} else {
		pScreen->DestroyPixmap(buf->pPixmap);
	}

	for (i = 0; i < buf->extraCount; i++) {
		pScreen->DestroyPixmap(buf->extraPix[i]);
	}

	free(buf->extraPix);
	free(buf->extraNames);
	free(buf->extraPitches);

	free(buf);
}

/**
 *
 */
static void
OMAPDRI2CopyRegion(DrawablePtr pDraw, RegionPtr pRegion,
		DRI2BufferPtr pDstBuffer, DRI2BufferPtr pSrcBuffer)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	DrawablePtr pSrcDraw = dri2draw(pDraw, pSrcBuffer);
	DrawablePtr pDstDraw = dri2draw(pDraw, pDstBuffer);
	RegionPtr pCopyClip;
	GCPtr pGC;

	DEBUG_MSG("pDraw=%p, pDstBuffer=%p (%p), pSrcBuffer=%p (%p)",
			pDraw, pDstBuffer, pSrcDraw, pSrcBuffer, pDstDraw);

	if (pDstBuffer->attachment == DRI2BufferFakeFrontLeft) {
		/* I think we don't need this.. it should be handled
		 * on client side (I think)..
		 */
		return;
	}

	pGC = GetScratchGC(pDstDraw->depth, pScreen);
	if (!pGC) {
		return;
	}

	pCopyClip = RegionCreate(NULL, 0);
	RegionCopy(pCopyClip, pRegion);
	(*pGC->funcs->ChangeClip) (pGC, CT_REGION, pCopyClip, 0);
	ValidateGC(pDstDraw, pGC);

	/* If the dst is the framebuffer, and we had a way to
	 * schedule a deferred blit synchronized w/ vsync, that
	 * would be a nice thing to do utilize here to avoid
	 * tearing..  when we have sync object support for GEM
	 * buffers, I think we could do something more clever
	 * here.
	 */

	pGC->ops->CopyArea(pSrcDraw, pDstDraw, pGC,
			0, 0, pDraw->width, pDraw->height, 0, 0);

	FreeScratchGC(pGC);
}

/**
 * Get current frame count and frame count timestamp, based on drawable's
 * crtc.
 */
static int
OMAPDRI2GetMSC(DrawablePtr pDraw, CARD64 *ust, CARD64 *msc)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	drmVBlank vbl = { .request = {
		.type = DRM_VBLANK_RELATIVE,
		.sequence = 0,
	} };
	int ret;

	ret = drmWaitVBlank(pOMAP->drmFD, &vbl);
	if (ret) {
		static int limit = 5;
		if (limit) {
			ERROR_MSG("get vblank counter failed: %s", strerror(errno));
			limit--;
		}
		return FALSE;
	}

	if (ust) {
		*ust = ((CARD64)vbl.reply.tval_sec * 1000000) + vbl.reply.tval_usec;
	}
	if (msc) {
		*msc = vbl.reply.sequence;
	}

	return TRUE;
}

struct _OMAPDRISwapCmd {
	int type;
	ClientPtr client;
	DrawablePtr pDraw;
	DRI2BufferPtr pDstBuffer;
	DRI2BufferPtr pSrcBuffer;
	DRI2SwapEventPtr func;
	void *data;
};

void
OMAPDRI2SwapComplete(OMAPDRISwapCmd *cmd)
{
	ScreenPtr pScreen = cmd->pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];

	DEBUG_MSG("%d -> %d", cmd->pSrcBuffer->attachment,
			cmd->pDstBuffer->attachment);

	if (cmd->type != DRI2_BLIT_COMPLETE)
		exchangebufs(cmd->pDraw, cmd->pSrcBuffer, cmd->pDstBuffer);

	DRI2SwapComplete(cmd->client, cmd->pDraw, 0, 0, 0,
			cmd->type, cmd->func, cmd->data);

	free(cmd);
}

/**
 * ScheduleSwap is responsible for requesting a DRM vblank event for the
 * appropriate frame.
 *
 * In the case of a blit (e.g. for a windowed swap) or buffer exchange,
 * the vblank requested can simply be the last queued swap frame + the swap
 * interval for the drawable.
 *
 * In the case of a page flip, we request an event for the last queued swap
 * frame + swap interval - 1, since we'll need to queue the flip for the frame
 * immediately following the received event.
 */
static int
OMAPDRI2ScheduleSwap(ClientPtr client, DrawablePtr pDraw,
		DRI2BufferPtr pDstBuffer, DRI2BufferPtr pSrcBuffer,
		CARD64 *target_msc, CARD64 divisor, CARD64 remainder,
		DRI2SwapEventPtr func, void *data)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPDRI2BufferPtr src = OMAPBUF(pSrcBuffer);
	OMAPDRI2BufferPtr dst = OMAPBUF(pDstBuffer);
	OMAPDRISwapCmd *cmd = calloc(1, sizeof(*cmd));

	cmd->client = client;
	cmd->pDraw = pDraw;
	cmd->pSrcBuffer = pSrcBuffer;
	cmd->pDstBuffer = pDstBuffer;
	cmd->func = func;
	cmd->data = data;

	DEBUG_MSG("%d -> %d", pSrcBuffer->attachment, pDstBuffer->attachment);

	if (src->fb_id && dst->fb_id) {
		DEBUG_MSG("can flip:  %d -> %d", src->fb_id, dst->fb_id);
		cmd->type = DRI2_FLIP_COMPLETE;
		drmmode_page_flip(pDraw, src->fb_id, cmd);
	} else if (canexchange(pDraw, pSrcBuffer, pDstBuffer)) {
		/* we can get away w/ pointer swap.. yah! */
		cmd->type = DRI2_EXCHANGE_COMPLETE;
		OMAPDRI2SwapComplete(cmd);
	} else {
		/* fallback to blit: */
		BoxRec box = {
				.x1 = 0,
				.y1 = 0,
				.x2 = pDraw->width,
				.y2 = pDraw->height,
		};
		RegionRec region;
		RegionInit(&region, &box, 0);
		OMAPDRI2CopyRegion(pDraw, &region, pDstBuffer, pSrcBuffer);
		cmd->type = DRI2_BLIT_COMPLETE;
		OMAPDRI2SwapComplete(cmd);
	}

	return TRUE;
}

static int OMAPDRI2PutTextureImage(
		PixmapPtr pSrcPix, BoxPtr pSrcBox,
		PixmapPtr pOsdPix, BoxPtr pOsdBox,
		PixmapPtr pDstPix, BoxPtr pDstBox,
		void *closure)
{
	ScreenPtr pScreen = pDstPix->drawable.pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	OMAPDRISwapCmd *cmd = closure;
	OMAPDRI2BufferPtr src = OMAPBUF(cmd->pSrcBuffer);
	Bool ret;

	ret = pOMAP->pOMAPEXA->PutTextureImage(pSrcPix, pSrcBox,
			pOsdPix, pOsdBox, pDstPix, pDstBox,
			src->extraCount, src->extraPix,
			cmd->pSrcBuffer->format);
	if (ret) {
		return Success;
	}
	DEBUG_MSG("PutTextureImage failed");

	return BadImplementation;
}


/**
 * Schedule a video buffer swap
 *
 * Drivers should queue an event for the frame count that satisfies the
 * parameters passed in.  If the event is in the future (i.e. the conditions
 * aren't currently satisfied), the server may block the client at the next
 * GLX request using DRI2WaitSwap. When the event arrives, drivers should call
 * \c DRI2SwapComplete, which will handle waking the client and returning
 * the appropriate data.
 *
 * The DDX is responsible for doing an overlay buffer flip/exchange, or
 * scaling/colorconvert blit when the corresponding event arrives.
 *
 * If the target drawable is resized/damaged, or the osd pixmap is changed/
 * damaged, ScheduleSwapVid can be re-invoked by the core with the same
 * source buffer to repair the dri2 video drawable.
 */
static Bool
OMAPDRI2ScheduleSwapVid(ClientPtr client, DrawablePtr pDraw,
		DRI2BufferPtr pDstBuffer, DRI2BufferPtr pSrcBuffer,
		BoxPtr pSrcBox, DrawablePtr osd,
		CARD64 *target_msc, CARD64 divisor, CARD64 remainder,
		DRI2SwapEventPtr func, void *data)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPDRISwapCmd *cmd = calloc(1, sizeof(*cmd));
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	RegionPtr pCopyClip;
	GCPtr pGC;
	BoxRec dstbox = {
			.x1 = pDraw->x, .y1 = pDraw->y,
			.x2 = pDraw->x + pDraw->width, .y2 = pDraw->y + pDraw->height
	};
	BoxRec osdbox = {
			.x1 = 0, .y1 = 0,
			.x2 = osd ? osd->width : 0, .y2 = osd ? osd->height : 0,
	};

	cmd->client = client;
	cmd->pDraw = pDraw;
	cmd->pSrcBuffer = pSrcBuffer;
	cmd->pDstBuffer = pDstBuffer;
	cmd->func = func;
	cmd->data = data;
	cmd->type = DRI2_BLIT_COMPLETE;

	DEBUG_MSG("%d -> %d", pSrcBuffer->attachment, pDstBuffer->attachment);

	pGC = GetScratchGC(pDraw->depth, pScreen);
	if (!pGC) {
		return FALSE;
	}

	pCopyClip = RegionCreate(&dstbox, 1);
	(*pGC->funcs->ChangeClip) (pGC, CT_REGION, pCopyClip, 0);
	ValidateGC(pDraw, pGC);

	/* someday, support overlay too.. */
	if (has_video(pOMAP)) {
		if (OMAPVidCopyArea(dri2draw(pDraw, pSrcBuffer), pSrcBox,
				osd, &osdbox, dri2draw(pDraw, pDstBuffer), &dstbox,
				OMAPDRI2PutTextureImage, cmd, pCopyClip) == Success) {
			OMAPDRI2SwapComplete(cmd);
			return TRUE;
		}
	}

	// XXX for RGB formats, we should be able to fall back to
	// generic composite operation..

	return FALSE;
}

/**
 * Request a DRM event when the requested conditions will be satisfied.
 *
 * We need to handle the event and ask the server to wake up the client when
 * we receive it.
 */
static int
OMAPDRI2ScheduleWaitMSC(ClientPtr client, DrawablePtr pDraw, CARD64 target_msc,
		CARD64 divisor, CARD64 remainder)
{
	ScreenPtr pScreen = pDraw->pScreen;
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
//	OMAPPtr pOMAP = OMAPPTR(pScrn);

#if 0
#endif
	ERROR_MSG("not implemented");
	return FALSE;
}

#define ATOM(a) MakeAtom(a, sizeof(a) - 1, TRUE)

static int
OMAPDRI2SetAttribute(DrawablePtr pDraw, Atom attribute,
		int len, const CARD32 *val)
{
	/* just for testing.. bogus colorspace conversion matrix.. */
	if (attribute == ATOM("XV_CSC_MATRIX")) {
		return Success;
	}
	return BadMatch;
}

static int
OMAPDRI2GetAttribute(DrawablePtr pDraw, Atom attribute,
		int *len, const CARD32 **val)
{
	/* just for testing.. bogus colorspace conversion matrix.. */
	if (attribute == ATOM("XV_CSC_MATRIX")) {
		static const CARD32 csc[] = {
				0x00, 0x01, 0x02, 0x03,
				0x10, 0x11, 0x12, 0x13,
				0x20, 0x21, 0x22, 0x23,
		};
		*val = csc;
		*len = sizeof(csc) / 4;
		return Success;
	}
	return BadMatch;
}

/**
 * The DRI2 ScreenInit() function.. register our handler fxns w/ DRI2 core
 */
Bool
OMAPDRI2ScreenInit(ScreenPtr pScreen)
{
	ScrnInfoPtr pScrn = xf86Screens[pScreen->myNum];
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	const char *driverNames[] = {
			[DRI2DriverDRI] = "omap",
			[DRI2DriverXV]  = has_video(pOMAP) ? "omap" : NULL,
	};
	DRI2InfoRec info = {
			.version			= 7,
			.fd 				= pOMAP->drmFD,
			.numDrivers			= 3,
			.driverNames		= driverNames,
			.driverName			= driverNames[DRI2DriverDRI],
			.deviceName			= pOMAP->deviceName,
			.CreateBuffer		= OMAPDRI2CreateBuffer,
			.CreateBufferVid	= OMAPDRI2CreateBufferVid,
			.DestroyBuffer		= OMAPDRI2DestroyBuffer,
			.CopyRegion			= OMAPDRI2CopyRegion,
			.ScheduleSwap		= OMAPDRI2ScheduleSwap,
			.ScheduleSwapVid	= OMAPDRI2ScheduleSwapVid,
			.ScheduleWaitMSC	= OMAPDRI2ScheduleWaitMSC,
			.GetMSC				= OMAPDRI2GetMSC,
			.AuthMagic			= drmAuthMagic,
			.GetExtraBufferNames= OMAPDRI2GetExtraBufferNames,
			.SetAttribute		= OMAPDRI2SetAttribute,
			.GetAttribute		= OMAPDRI2GetAttribute,
	};
	int minor = 1, major = 0, nformats = 0;
	static unsigned int formats[MAX_FORMATS + 4];

	if (has_video(pOMAP)) {
		/* add supported video formats: */
		nformats = pOMAP->pOMAPEXA->GetFormats(formats);
	}

	/* add supported non-video formats: */
	formats[nformats++] = 24;
	formats[nformats++] = 32;
	formats[nformats++] = fourcc_code('X','R','2','4');
	formats[nformats++] = fourcc_code('A','R','2','4');

	info.formats = formats;
	info.numFormats = nformats;

	if (xf86LoaderCheckSymbol("DRI2Version")) {
		DRI2Version(&major, &minor);
	}

	if (minor < 1) {
		WARNING_MSG("DRI2 requires DRI2 module version 1.1.0 or later");
		return FALSE;
	}

	return DRI2ScreenInit(pScreen, &info);
}

/**
 * The DRI2 CloseScreen() function.. unregister ourself w/ DRI2 core.
 */
void
OMAPDRI2CloseScreen(ScreenPtr pScreen)
{
	DRI2CloseScreen(pScreen);
}
