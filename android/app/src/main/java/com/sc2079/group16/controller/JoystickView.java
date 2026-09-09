package com.sc2079.group16.controller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

/**
 * Circular touch joystick. Exposes the current knob offset as a normalized
 * (right, forward) vector in [-1, 1] via {@link #getStickRight()}/{@link
 * #getStickForward()} -- callers poll these rather than being pushed events,
 * because a joystick held still at a deflected position produces no further
 * {@link MotionEvent}s but should still read as deflected.
 */
class JoystickView extends View {

    private final Paint basePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint knobPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float centerX;
    private float centerY;
    private float baseRadius;
    private float knobRadius;

    private volatile float normRight = 0f;
    private volatile float normForward = 0f;

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        // Colors ported from facebook/astryx's "neutral" theme -- see
        // res/values/colors.xml. Resolved once here rather than via
        // android:color XML attrs since this View draws itself on a Canvas.
        basePaint.setColor(context.getColor(R.color.border_emphasized));
        knobPaint.setColor(context.getColor(R.color.accent));
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        centerX = w / 2f;
        centerY = h / 2f;
        baseRadius = Math.min(w, h) / 2f - 8f;
        knobRadius = baseRadius * 0.4f;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawCircle(centerX, centerY, baseRadius, basePaint);
        canvas.drawCircle(
                centerX + normRight * (baseRadius - knobRadius),
                centerY - normForward * (baseRadius - knobRadius), // screen Y grows downward
                knobRadius,
                knobPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_MOVE: {
                float dx = event.getX() - centerX;
                float dy = event.getY() - centerY;
                float mag = (float) Math.hypot(dx, dy);
                if (mag > baseRadius) {
                    dx = dx / mag * baseRadius;
                    dy = dy / mag * baseRadius;
                }
                normRight = dx / baseRadius;
                normForward = -dy / baseRadius;
                invalidate();
                return true;
            }
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                normRight = 0f;
                normForward = 0f;
                invalidate();
                return true;
            default:
                return super.onTouchEvent(event);
        }
    }

    /** [-1, 1], positive = pushed right. */
    float getStickRight() {
        return normRight;
    }

    /** [-1, 1], positive = pushed toward the top (forward). */
    float getStickForward() {
        return normForward;
    }
}
