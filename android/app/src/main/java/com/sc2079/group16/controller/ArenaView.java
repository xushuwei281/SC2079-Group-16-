package com.sc2079.group16.controller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 2D top-down display of the 200x200cm arena: obstacles (10x10cm, each with
 * an ID and a marked target face) and the robot's live pose. Coordinate
 * convention matches the rest of the system (see ARCHITECTURE.md /
 * serial_bridge_node.py): origin at the arena's bottom-left corner, +X =
 * East, +Y = North, heading in degrees CCW from East.
 *
 * Touch interaction (see class doc in MainActivity for the full writeup):
 *   - drag an obstacle to reposition it; release outside the arena deletes it
 *   - tap (drag distance below a threshold) cycles its marked face N->E->S->W
 * Obstacle layout is transmitted by the caller via {@link #buildAlgCommand()},
 * which matches mdp_bringup/planner_node.py's ALG|id,x,y,face|... parser
 * exactly -- no Pi-side changes needed.
 */
class ArenaView extends View {

    static final int ARENA_SIZE_CM = 200;
    static final int OBSTACLE_SIZE_CM = 10;
    static final int ROBOT_SIZE_CM = 20;
    private static final int GRID_CELLS = 10; // 20cm per cell

    private static final char[] FACE_CYCLE = {'N', 'E', 'S', 'W'};

    static class Obstacle {
        final int id;
        float xCm;
        float yCm;
        char face;
        Integer recognizedSymbol; // set once a TARGET,<id>,<symbol> line arrives

        Obstacle(int id, float xCm, float yCm, char face) {
            this.id = id;
            this.xCm = xCm;
            this.yCm = yCm;
            this.face = face;
        }
    }

    public interface ObstacleListener {
        void onObstacleTapped(Obstacle obstacle, float canvasPxX, float canvasPxY, float widthPx, float heightPx);
        void onObstacleMoved(Obstacle obstacle);
        void onObstacleDeleted(Obstacle obstacle);
    }

    private ObstacleListener obstacleListener;

    public void setObstacleListener(ObstacleListener listener) {
        this.obstacleListener = listener;
    }

    private final List<Obstacle> obstacles = new ArrayList<>();
    private int nextObstacleId = 1;

    // Robot's live pose, defaulting to the start-zone center/heading
    // serial_bridge_node.py itself defaults to (see its initial_x/y/yaw).
    private float robotXCm = 20f;
    private float robotYCm = 20f;
    private float robotHeadingDeg = 90f; // North

    private float arenaLeftPx;
    private float arenaTopPx;
    private float arenaPixelSize;
    private float minTouchRadiusPx;

    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint obstaclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint obstacleFacePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint obstacleTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint robotHeadingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint axisLabelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private static final int AXIS_LABEL_STEP_CM = 10; // marks at 10, 20, ..., 200
    private static final float DRAG_MARGIN_CM = 15f; // Boundary margin allowing visual feedback for deletion outside arena

    private Obstacle draggingObstacle;
    private int activePointerId = MotionEvent.INVALID_POINTER_ID;
    private float dragDownPx;
    private float dragDownPy;
    private float dragTotalMovementPx;

    ArenaView() {
        super(null);
    }

    public ArenaView(Context context, AttributeSet attrs) {
        super(context, attrs);
        // Colors ported from facebook/astryx's "neutral" theme -- see
        // res/values/colors.xml. Resolved once here rather than via
        // android:color XML attrs since this View draws itself on a Canvas.
        gridPaint.setColor(context.getColor(R.color.border));
        gridPaint.setStrokeWidth(1f);
        borderPaint.setColor(context.getColor(R.color.border_emphasized));
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(4f);
        obstaclePaint.setColor(context.getColor(R.color.categorical_orange_icon));
        obstacleFacePaint.setColor(context.getColor(R.color.status_error_fill)); // marks the target face
        obstacleTextPaint.setColor(context.getColor(R.color.badge_text)); // high contrast against orange fill in both themes
        obstacleTextPaint.setTextAlign(Paint.Align.CENTER);
        robotPaint.setColor(context.getColor(R.color.status_accent_fill)); // reserved for live/info markers
        robotHeadingPaint.setColor(context.getColor(R.color.on_info));
        robotHeadingPaint.setStrokeWidth(6f);
        axisLabelPaint.setColor(context.getColor(R.color.text_secondary));

        minTouchRadiusPx = 24f * context.getResources().getDisplayMetrics().density;
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        arenaPixelSize = Math.min(w, h);
        arenaLeftPx = (w - arenaPixelSize) / 2f;
        arenaTopPx = (h - arenaPixelSize) / 2f;
        obstacleTextPaint.setTextSize(arenaPixelSize * 0.03f);
        axisLabelPaint.setTextSize(arenaPixelSize * 0.018f);
    }

    // ---- Public API, called from MainActivity ------------------------------

    void addObstacle() {
        obstacles.add(new Obstacle(nextObstacleId++, ARENA_SIZE_CM / 2f, ARENA_SIZE_CM / 2f, 'N'));
        invalidate();
    }

    void clearObstacles() {
        draggingObstacle = null;
        activePointerId = MotionEvent.INVALID_POINTER_ID;
        obstacles.clear();
        nextObstacleId = 1;
        invalidate();
    }

    int getObstacleCount() {
        return obstacles.size();
    }

    void setObstacleFace(int obstacleId, char face) {
        for (Obstacle o : obstacles) {
            if (o.id == obstacleId) {
                o.face = face;
                invalidate();
                return;
            }
        }
    }

    void removeObstacle(int obstacleId) {
        if (draggingObstacle != null && draggingObstacle.id == obstacleId) {
            draggingObstacle = null;
            activePointerId = MotionEvent.INVALID_POINTER_ID;
        }
        for (int i = 0; i < obstacles.size(); i++) {
            if (obstacles.get(i).id == obstacleId) {
                Obstacle removed = obstacles.remove(i);
                if (obstacleListener != null) {
                    obstacleListener.onObstacleDeleted(removed);
                }
                invalidate();
                return;
            }
        }
    }

    /** Builds the exact ALG|id,x,y,face|... string planner_node.py parses. */
    String buildAlgCommand() {
        return buildAlgCommand(this.obstacles);
    }

    static String buildAlgCommand(List<Obstacle> obstacles) {
        StringBuilder sb = new StringBuilder("ALG");
        for (Obstacle o : obstacles) {
            sb.append('|')
                    .append(o.id)
                    .append(',')
                    .append(Math.round(o.xCm))
                    .append(',')
                    .append(Math.round(o.yCm))
                    .append(',')
                    .append(o.face);
        }
        return sb.toString();
    }

    void setRobotPose(float xCm, float yCm, float headingDeg) {
        robotXCm = xCm;
        robotYCm = yCm;
        robotHeadingDeg = headingDeg;
        invalidate();
    }

    void setRecognizedSymbol(int obstacleId, int symbolId) {
        for (Obstacle o : obstacles) {
            if (o.id == obstacleId) {
                o.recognizedSymbol = symbolId;
                invalidate();
                return;
            }
        }
    }

    // ---- Coordinate transforms (cm, origin bottom-left) <-> pixels --------

    private float cmXToPx(float xCm) {
        return arenaLeftPx + (xCm / ARENA_SIZE_CM) * arenaPixelSize;
    }

    private float cmYToPx(float yCm) {
        return arenaTopPx + arenaPixelSize - (yCm / ARENA_SIZE_CM) * arenaPixelSize;
    }

    private float pxToXCm(float px) {
        return (px - arenaLeftPx) / arenaPixelSize * ARENA_SIZE_CM;
    }

    private float pxToYCm(float py) {
        return (arenaTopPx + arenaPixelSize - py) / arenaPixelSize * ARENA_SIZE_CM;
    }

    // ---- Drawing ------------------------------------------------------------

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        for (int i = 0; i <= GRID_CELLS; i++) {
            float x = cmXToPx(i * (ARENA_SIZE_CM / (float) GRID_CELLS));
            canvas.drawLine(x, arenaTopPx, x, arenaTopPx + arenaPixelSize, gridPaint);
            float y = cmYToPx(i * (ARENA_SIZE_CM / (float) GRID_CELLS));
            canvas.drawLine(arenaLeftPx, y, arenaLeftPx + arenaPixelSize, y, gridPaint);
        }
        canvas.drawRect(arenaLeftPx, arenaTopPx, arenaLeftPx + arenaPixelSize, arenaTopPx + arenaPixelSize, borderPaint);
        drawAxisLabels(canvas);

        for (Obstacle o : obstacles) {
            drawObstacle(canvas, o);
        }

        drawRobot(canvas);
    }

    /** Cm scale marks along the bottom (X) and left (Y) edges, at
     * AXIS_LABEL_STEP_CM intervals -- lets you read an obstacle's
     * approximate position off the grid without tapping it. */
    private void drawAxisLabels(Canvas canvas) {
        float padding = axisLabelPaint.getTextSize() * 0.9f;

        axisLabelPaint.setTextAlign(Paint.Align.CENTER);
        for (int xCm = AXIS_LABEL_STEP_CM; xCm <= ARENA_SIZE_CM; xCm += AXIS_LABEL_STEP_CM) {
            canvas.drawText(
                    String.valueOf(xCm),
                    cmXToPx(xCm),
                    arenaTopPx + arenaPixelSize - padding * 0.3f,
                    axisLabelPaint);
        }

        axisLabelPaint.setTextAlign(Paint.Align.LEFT);
        for (int yCm = AXIS_LABEL_STEP_CM; yCm <= ARENA_SIZE_CM; yCm += AXIS_LABEL_STEP_CM) {
            canvas.drawText(
                    String.valueOf(yCm),
                    arenaLeftPx + padding * 0.2f,
                    cmYToPx(yCm) + axisLabelPaint.getTextSize() * 0.35f,
                    axisLabelPaint);
        }
    }

    private void drawObstacle(Canvas canvas, Obstacle o) {
        float half = OBSTACLE_SIZE_CM / 2f;
        float left = cmXToPx(o.xCm - half);
        float right = cmXToPx(o.xCm + half);
        float top = cmYToPx(o.yCm + half); // +Y is up on screen
        float bottom = cmYToPx(o.yCm - half);

        canvas.drawRect(left, top, right, bottom, obstaclePaint);

        float faceThickness = (right - left) * 0.22f;
        switch (o.face) {
            case 'N':
                canvas.drawRect(left, top, right, top + faceThickness, obstacleFacePaint);
                break;
            case 'S':
                canvas.drawRect(left, bottom - faceThickness, right, bottom, obstacleFacePaint);
                break;
            case 'E':
                canvas.drawRect(right - faceThickness, top, right, bottom, obstacleFacePaint);
                break;
            case 'W':
                canvas.drawRect(left, top, left + faceThickness, bottom, obstacleFacePaint);
                break;
            default:
                break;
        }

        String label = o.recognizedSymbol != null
                ? String.format(Locale.US, "%d\n#%d", o.id, o.recognizedSymbol)
                : String.valueOf(o.id);
        float centerX = (left + right) / 2f;
        float centerY = (top + bottom) / 2f + obstacleTextPaint.getTextSize() * 0.35f;
        canvas.drawText(label, centerX, centerY, obstacleTextPaint);
    }

    private void drawRobot(Canvas canvas) {
        float cx = cmXToPx(robotXCm);
        float cy = cmYToPx(robotYCm);
        float radiusPx = (ROBOT_SIZE_CM / 2f) / ARENA_SIZE_CM * arenaPixelSize;
        canvas.drawCircle(cx, cy, radiusPx, robotPaint);

        double headingRad = Math.toRadians(robotHeadingDeg);
        float noseXCm = robotXCm + (ROBOT_SIZE_CM / 2f) * (float) Math.cos(headingRad);
        float noseYCm = robotYCm + (ROBOT_SIZE_CM / 2f) * (float) Math.sin(headingRad);
        canvas.drawLine(cx, cy, cmXToPx(noseXCm), cmYToPx(noseYCm), robotHeadingPaint);
    }

    // ---- Touch: drag to move, tap to cycle face ----------------------------

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN: {
                activePointerId = event.getPointerId(0);
                int pointerIndex = event.findPointerIndex(activePointerId);
                if (pointerIndex == -1) {
                    draggingObstacle = null;
                    activePointerId = MotionEvent.INVALID_POINTER_ID;
                    return false;
                }
                dragDownPx = event.getX(pointerIndex);
                dragDownPy = event.getY(pointerIndex);
                draggingObstacle = findObstacleNear(dragDownPx, dragDownPy);
                dragTotalMovementPx = 0f;
                if (draggingObstacle == null) {
                    activePointerId = MotionEvent.INVALID_POINTER_ID;
                    return false;
                }
                return true;
            }
            case MotionEvent.ACTION_POINTER_DOWN: {
                // Secondary pointer touched down: maintain activePointerId to prevent
                // multi-touch gestures from hijacking or corrupting draggingObstacle state
                return draggingObstacle != null;
            }
            case MotionEvent.ACTION_MOVE: {
                if (draggingObstacle == null || activePointerId == MotionEvent.INVALID_POINTER_ID) {
                    return false;
                }
                int pointerIndex = event.findPointerIndex(activePointerId);
                if (pointerIndex == -1 || pointerIndex >= event.getPointerCount()) {
                    return false;
                }
                float x = event.getX(pointerIndex);
                float y = event.getY(pointerIndex);
                dragTotalMovementPx = (float) Math.hypot(x - dragDownPx, y - dragDownPy);

                float rawXCm = pxToXCm(x);
                float rawYCm = pxToYCm(y);
                // Clamp position reasonably within arena bounds plus a delete buffer
                // so the obstacle does not jump uncontrollably off-screen
                draggingObstacle.xCm = clampDragCm(rawXCm);
                draggingObstacle.yCm = clampDragCm(rawYCm);
                invalidate();
                return true;
            }
            case MotionEvent.ACTION_POINTER_UP: {
                if (draggingObstacle == null || activePointerId == MotionEvent.INVALID_POINTER_ID) {
                    return false;
                }
                int actionIndex = event.getActionIndex();
                int pointerId = event.getPointerId(actionIndex);
                if (pointerId == activePointerId) {
                    // Active dragging finger was lifted; commit drag/tap/delete
                    finishDrag();
                    return true;
                }
                // Secondary finger was lifted; continue dragging with active pointer
                return true;
            }
            case MotionEvent.ACTION_UP: {
                if (draggingObstacle == null) {
                    return false;
                }
                finishDrag();
                return true;
            }
            case MotionEvent.ACTION_CANCEL: {
                if (draggingObstacle != null) {
                    draggingObstacle = null;
                    activePointerId = MotionEvent.INVALID_POINTER_ID;
                    invalidate();
                    return true;
                }
                return false;
            }
            default:
                return super.onTouchEvent(event);
        }
    }

    private void finishDrag() {
        if (draggingObstacle == null) {
            activePointerId = MotionEvent.INVALID_POINTER_ID;
            return;
        }
        Obstacle obstacle = draggingObstacle;
        draggingObstacle = null;
        activePointerId = MotionEvent.INVALID_POINTER_ID;

        if (dragTotalMovementPx < minTouchRadiusPx) {
            if (obstacleListener != null) {
                float half = OBSTACLE_SIZE_CM / 2f;
                float leftPx = cmXToPx(obstacle.xCm - half);
                float topPx = cmYToPx(obstacle.yCm + half);
                float sizePx = (OBSTACLE_SIZE_CM / (float) ARENA_SIZE_CM) * arenaPixelSize;
                obstacleListener.onObstacleTapped(obstacle, leftPx, topPx, sizePx, sizePx);
            } else {
                obstacle.face = nextFace(obstacle.face);
            }
        } else if (isOutsideArena(obstacle.xCm, obstacle.yCm)) {
            obstacles.remove(obstacle);
            if (obstacleListener != null) {
                obstacleListener.onObstacleDeleted(obstacle);
            }
        } else {
            // Keep obstacle fully within arena bounds when placed
            obstacle.xCm = clampInsideArena(obstacle.xCm);
            obstacle.yCm = clampInsideArena(obstacle.yCm);
            if (obstacleListener != null) {
                obstacleListener.onObstacleMoved(obstacle);
            }
        }
        invalidate();
    }

    static float clampDragCm(float rawCm) {
        return Math.max(-DRAG_MARGIN_CM, Math.min(rawCm, ARENA_SIZE_CM + DRAG_MARGIN_CM));
    }

    static float clampInsideArena(float val) {
        float half = OBSTACLE_SIZE_CM / 2f;
        return Math.max(half, Math.min(val, ARENA_SIZE_CM - half));
    }

    private Obstacle findObstacleNear(float px, float py) {
        // Search newest-first so an overlapping obstacle you just placed is
        // the one that responds, not whatever's underneath it.
        for (int i = obstacles.size() - 1; i >= 0; i--) {
            Obstacle o = obstacles.get(i);
            float dx = px - cmXToPx(o.xCm);
            float dy = py - cmYToPx(o.yCm);
            float halfPx = Math.max(
                    (OBSTACLE_SIZE_CM / 2f) / ARENA_SIZE_CM * arenaPixelSize, minTouchRadiusPx);
            if (Math.abs(dx) <= halfPx && Math.abs(dy) <= halfPx) {
                return o;
            }
        }
        return null;
    }

    static boolean isOutsideArena(float xCm, float yCm) {
        return xCm < 0 || xCm > ARENA_SIZE_CM || yCm < 0 || yCm > ARENA_SIZE_CM;
    }

    static char nextFace(char face) {
        for (int i = 0; i < FACE_CYCLE.length; i++) {
            if (FACE_CYCLE[i] == face) {
                return FACE_CYCLE[(i + 1) % FACE_CYCLE.length];
            }
        }
        return 'N';
    }
}
