package com.sc2079.group16.controller;

import android.app.Activity;
import android.content.Context;
import android.content.ContextWrapper;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Build;
import android.util.DisplayMetrics;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowInsets;
import android.widget.PopupWindow;

/**
 * Native Android implementation of Facebook Astryx's Core Layer positioning
 * hook ({@code useLayer} / {@code Layer}).
 *
 * Provides anchor positioning with configurable clearance offset (defaults to
 * 12dp matching Storybook's {@code core-layer--offset}), boundary auto-flipping,
 * and light-dismissal ({@code lightDismiss: true}).
 *
 * @see <a href="https://facebook.github.io/astryx/storybook/index.html?path=/story/core-layer--offset">Astryx Layer Offset</a>
 */
public class AstryxLayer {

    public enum Placement {
        ABOVE,
        BELOW,
        START,
        END
    }

    public static final float DEFAULT_OFFSET_DP = 12f;

    private final Context context;
    private final PopupWindow popupWindow;
    private View contentView;
    private Placement placement = Placement.ABOVE;
    private float offsetDp = DEFAULT_OFFSET_DP; // Default 12dp clearance offset from Astryx spec
    private boolean lightDismiss = true;

    AstryxLayer() {
        this.context = null;
        this.popupWindow = null;
    }

    public AstryxLayer(Context context) {
        this.context = context;
        this.popupWindow = new PopupWindow(context);
        this.popupWindow.setWidth(ViewGroup.LayoutParams.WRAP_CONTENT);
        this.popupWindow.setHeight(ViewGroup.LayoutParams.WRAP_CONTENT);
        this.popupWindow.setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
        this.popupWindow.setElevation(8f * context.getResources().getDisplayMetrics().density);
        setLightDismiss(true);
    }

    public void setContentView(View view) {
        this.contentView = view;
        if (this.popupWindow != null) {
            this.popupWindow.setContentView(view);
        }
    }

    public View getContentView() {
        return contentView;
    }

    public void setPlacement(Placement placement) {
        this.placement = placement;
    }

    public Placement getPlacement() {
        return placement;
    }

    public void setOffsetDp(float offsetDp) {
        this.offsetDp = offsetDp;
    }

    public float getOffsetDp() {
        return offsetDp;
    }

    public void setLightDismiss(boolean lightDismiss) {
        this.lightDismiss = lightDismiss;
        if (this.popupWindow != null) {
            this.popupWindow.setFocusable(lightDismiss);
            this.popupWindow.setOutsideTouchable(lightDismiss);
        }
    }

    public void setOnDismissListener(PopupWindow.OnDismissListener listener) {
        if (this.popupWindow != null) {
            this.popupWindow.setOnDismissListener(listener);
        }
    }

    public boolean isShowing() {
        return popupWindow != null && popupWindow.isShowing();
    }

    public void dismiss() {
        if (context == null || popupWindow == null) return;
        Activity activity = getActivity(context);
        if (activity != null && (activity.isFinishing() || activity.isDestroyed())) {
            return;
        }
        try {
            if (popupWindow.isShowing()) {
                popupWindow.dismiss();
            }
        } catch (IllegalArgumentException | IllegalStateException ignored) {
            // Android can throw IllegalArgumentException if the window token has already been detached
        }
    }

    /**
     * Anchors the layer relative to a parent view with exact canvas coordinates
     * (e.g. an obstacle or robot position in ArenaView).
     *
     * @param parentView Anchor's parent view.
     * @param anchorX The X coordinate of the anchor in parentView pixels.
     * @param anchorY The Y coordinate of the anchor in parentView pixels.
     * @param anchorWidth The width of the anchor in pixels.
     * @param anchorHeight The height of the anchor in pixels.
     */
    public void showAtCoordinates(View parentView, float anchorX, float anchorY, float anchorWidth, float anchorHeight) {
        if (contentView == null || parentView == null || context == null || popupWindow == null) return;

        Activity activity = getActivity(context);
        if (activity != null && (activity.isFinishing() || activity.isDestroyed())) {
            return;
        }

        if (!parentView.isAttachedToWindow() || parentView.getWindowToken() == null) {
            return;
        }

        DisplayMetrics dm = context.getResources().getDisplayMetrics();
        float density = dm.density;
        float offsetPx = offsetDp * density;

        // Measure content view dimensions
        contentView.measure(
                View.MeasureSpec.makeMeasureSpec(dm.widthPixels, View.MeasureSpec.AT_MOST),
                View.MeasureSpec.makeMeasureSpec(dm.heightPixels, View.MeasureSpec.AT_MOST));
        int popoverWidth = contentView.getMeasuredWidth();
        int popoverHeight = contentView.getMeasuredHeight();

        // Convert parent coordinates to screen window coordinates
        int[] parentScreenPos = new int[2];
        parentView.getLocationOnScreen(parentScreenPos);

        float screenAnchorX = parentScreenPos[0] + anchorX;
        float screenAnchorY = parentScreenPos[1] + anchorY;

        int edgeMargin = Math.round(12 * density);

        int topInset = 0;
        int bottomInset = 0;
        int leftInset = 0;
        int rightInset = 0;

        WindowInsets insets = parentView.getRootWindowInsets();
        if (insets != null) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                android.graphics.Insets barInsets = insets.getInsets(
                        WindowInsets.Type.systemBars() | WindowInsets.Type.displayCutout());
                topInset = barInsets.top;
                bottomInset = barInsets.bottom;
                leftInset = barInsets.left;
                rightInset = barInsets.right;
            } else {
                topInset = insets.getSystemWindowInsetTop();
                bottomInset = insets.getSystemWindowInsetBottom();
                leftInset = insets.getSystemWindowInsetLeft();
                rightInset = insets.getSystemWindowInsetRight();
            }
        }

        // Establish safe display boundaries accounting for system bars, cutouts, and edge margins
        int safeTop = Math.max(topInset, Math.round(24 * density)) + edgeMargin;
        int safeBottom = dm.heightPixels - Math.max(bottomInset, Math.round(48 * density)) - edgeMargin;
        int safeLeft = Math.max(leftInset, 0) + edgeMargin;
        int safeRight = dm.widthPixels - Math.max(rightInset, 0) - edgeMargin;

        if (safeBottom < safeTop) safeBottom = dm.heightPixels - edgeMargin;
        if (safeRight < safeLeft) safeRight = dm.widthPixels - edgeMargin;

        Placement effectivePlacement = this.placement;

        // Auto-flipping boundary logic: check vertical clearance
        if (effectivePlacement == Placement.ABOVE) {
            float targetY = screenAnchorY - popoverHeight - offsetPx;
            if (targetY < safeTop) { // near top edge/status bar: flip BELOW
                effectivePlacement = Placement.BELOW;
            }
        } else if (effectivePlacement == Placement.BELOW) {
            float targetY = screenAnchorY + anchorHeight + offsetPx;
            if (targetY + popoverHeight > safeBottom) { // near bottom/navigation bar: flip ABOVE
                effectivePlacement = Placement.ABOVE;
            }
        } else if (effectivePlacement == Placement.START) {
            float targetX = screenAnchorX - popoverWidth - offsetPx;
            if (targetX < safeLeft) {
                effectivePlacement = Placement.END;
            }
        } else if (effectivePlacement == Placement.END) {
            float targetX = screenAnchorX + anchorWidth + offsetPx;
            if (targetX + popoverWidth > safeRight) {
                effectivePlacement = Placement.START;
            }
        }

        // Compute final screen coordinates
        int posX;
        int posY;

        if (effectivePlacement == Placement.ABOVE) {
            float centerX = screenAnchorX + (anchorWidth / 2f);
            posX = Math.round(centerX - (popoverWidth / 2f));
            posY = Math.round(screenAnchorY - popoverHeight - offsetPx);
        } else if (effectivePlacement == Placement.BELOW) {
            float centerX = screenAnchorX + (anchorWidth / 2f);
            posX = Math.round(centerX - (popoverWidth / 2f));
            posY = Math.round(screenAnchorY + anchorHeight + offsetPx);
        } else if (effectivePlacement == Placement.START) {
            posX = Math.round(screenAnchorX - popoverWidth - offsetPx);
            float centerY = screenAnchorY + (anchorHeight / 2f);
            posY = Math.round(centerY - (popoverHeight / 2f));
        } else { // Placement.END
            posX = Math.round(screenAnchorX + anchorWidth + offsetPx);
            float centerY = screenAnchorY + (anchorHeight / 2f);
            posY = Math.round(centerY - (popoverHeight / 2f));
        }

        // Screen boundary clamping: strictly guarantee popover never renders partially
        // offscreen or beneath system navigation bars / status bars
        int minX = safeLeft;
        int maxX = Math.max(minX, safeRight - popoverWidth);
        posX = Math.max(minX, Math.min(posX, maxX));

        int minY = safeTop;
        int maxY = Math.max(minY, safeBottom - popoverHeight);
        posY = Math.max(minY, Math.min(posY, maxY));

        try {
            if (popupWindow.isShowing()) {
                popupWindow.update(posX, posY, popoverWidth, popoverHeight);
            } else {
                popupWindow.showAtLocation(parentView, Gravity.NO_GRAVITY, posX, posY);
            }
        } catch (Exception ignored) {
            // Guard against WindowManager.BadTokenException or IllegalArgumentException
            // if window token is detached between checks and dispatch
        }
    }

    private static Activity getActivity(Context context) {
        while (context instanceof ContextWrapper) {
            if (context instanceof Activity) {
                return (Activity) context;
            }
            context = ((ContextWrapper) context).getBaseContext();
        }
        return null;
    }
}
