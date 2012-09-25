#ifndef COMPAT_XWAYLAND_H
#define COMPAT_XWAYLAND_H

#include "xorg-server.h"

#define xorgWayland 0


struct xwl_screen;
struct xwl_window;

struct xwl_driver {
	int version;
	int use_drm;
	int (*create_window_buffer) (struct xwl_window *window, PixmapPtr pix);
};

static inline int
xwl_drm_authenticate(struct xwl_screen *xwl_screen, uint32_t magic)
{
	return 0;
}

static inline struct xwl_screen *
xwl_screen_create (void)
{
	return NULL;
}

static inline int
xwl_drm_pre_init(struct xwl_screen *xwl_screen)
{
	return 0;
}

static inline int
xwl_screen_get_drm_fd(struct xwl_screen *xwl_screen)
{
	return -1;
}

static inline void
xwl_screen_destroy(struct xwl_screen *xwl_screen)
{
}

static inline void
xwl_screen_post_damage(struct xwl_screen *xwl_screen)
{
}

static inline int
xwl_screen_init(struct xwl_screen *xwl_screen, ScreenPtr pScreen)
{
	return 0;
}

static inline void
xwl_screen_close(struct xwl_screen *xwl_screen)
{
}

static inline int
xwl_screen_pre_init(ScrnInfoPtr pScrn, struct xwl_screen *xwl_screen, int flags, struct xwl_driver *driver)
{
	return 0;
}

static inline int
xwl_create_window_buffer_drm(struct xwl_window *xwl_window, PixmapPtr pixmap, int name)
{
	return 0;
}

#endif /* COMPAT_XWAYLAND_H */
