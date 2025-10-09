#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#if defined(PLATFORM_WEB)
    #define CUSTOM_MODAL_DIALOGS            // Force custom modal dialogs usage
    #include <emscripten/emscripten.h>      // Emscripten library - LLVM to JavaScript compiler
#endif

typedef enum {
    NONE = 0, SURFACE, EDGE, POINT, INVALID
} colType;

typedef struct {
    colType type;
    Vector3 tri[3];
    Vector3 closestPoint;
    Vector3 shellPoint;
    Vector3 newSpherePos;

    //Edge
    Vector3 ep1;
    Vector3 ep2;
    Vector3 colPlaneTangent;
    Vector3 colPlaneShell;
    Vector3 planePoints[4];
    Vector3 colPlaneIntPoint;
    Vector3 newEdgePoint;
    Vector3 newShellPoint;

    //surface
    Vector3 newTriClosestPoint;

} triColEvent;

void DrawArrow(Vector3 start, Vector3 end, float size, Color color) {
    Vector3 dir = Vector3Normalize(Vector3Subtract(end,start));
    float length = Vector3Distance(start,end);
    Vector3 arrowStart = Vector3Add(start,Vector3Scale(dir, length*0.8f));
    DrawCylinderEx(arrowStart, end,size*2,0.00f,10,color);
    DrawCylinderEx(start, arrowStart,size,size,10,color);
}

bool rayPlaneIntersect(Vector3 rayOrigin, Vector3 rayDir, Vector3 planeOrigin, Vector3 planeNormal, float* t) {
    *t = 0.0f;
    float denom = Vector3DotProduct(planeNormal, rayDir);
    if (denom == 0) {
        return false;
    }
    float d = -Vector3DotProduct(planeNormal, planeOrigin);
    float numer = Vector3DotProduct(planeNormal, rayOrigin) + d;

    *t = -(numer / denom);
    return true;
}

Vector3 getTriangleNormal(Vector3 v1, Vector3 v2, Vector3 v3) {
    Vector3 edge1 = { v2.x - v1.x, v2.y - v1.y, v2.z - v1.z };
    Vector3 edge2 = { v3.x - v1.x, v3.y - v1.y, v3.z - v1.z };
    Vector3 normal = Vector3CrossProduct(edge1, edge2);
    float length = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    normal.x /= length;
    normal.y /= length;
    normal.z /= length;
    return normal;
}

Vector3 nearestPointOnLine(Vector3 l1, Vector3 l2, Vector3 p, bool* edge) {
    Vector3 lineDirection = Vector3Normalize(Vector3Subtract(l2, l1));
    Vector3 pointDirection = Vector3Subtract(p, l1);

    *edge = false;

    float distance = Vector3Distance(l1, l2);
    //float distance = Vector3DotProduct(pointDirection, lineDirection);
    float t = Vector3DotProduct(pointDirection, lineDirection);
    if (t < 0.0f) {
        return l1;
    }
    if (t > distance) {
        return l2;
    }
     *edge = true;
    return Vector3Add(l1, Vector3Scale(lineDirection, t));
}

typedef struct {
    colType collisionType;
    Vector3 pos;
    Vector3 edgePoint1;
    Vector3 edgePoint2;
}closestPointOnTri;

closestPointOnTri closestPointTriangle(Vector3 origin, Vector3 p1, Vector3 p2, Vector3 p3) {

    closestPointOnTri result;

    result.collisionType = NONE;

    Vector3 planeNormal = getTriangleNormal(p1, p2, p3);
    Vector3 inverseNormal = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, planeNormal);

    float intersectTime = 0.0f;
    rayPlaneIntersect(origin, inverseNormal, p1, planeNormal, &intersectTime);
    Vector3 OriginintersectionPoint = { origin.x + intersectTime * inverseNormal.x,
                                    origin.y + intersectTime * inverseNormal.y,
                                    origin.z + intersectTime * inverseNormal.z };

    float OrigindistToPlane = Vector3Distance(origin, OriginintersectionPoint);

    //Vector3 OriginintersectionPoint;
    //Ray OriginintersectionRay = { origin ,Vector3Normalize(inverseNormal) };
    //bool Originhit = rayPlaneCollision(OriginintersectionRay, p1, p2, p3, &OriginintersectionPoint);
    //float OrigindistToPlane = Vector3Distance(origin, OriginintersectionPoint);

    Vector3 closestPoint;
    Vector3 baryPoint = Vector3Barycenter(OriginintersectionPoint, p1, p2, p3);

    //if point inside triangle
    if (baryPoint.x >= 0 && baryPoint.y >= 0 && baryPoint.x + baryPoint.y <= 1) {
        closestPoint = OriginintersectionPoint;
        result.collisionType = SURFACE;
        result.edgePoint1 = p1;
        result.edgePoint2 = p1;
    }
    else {
        // Find nearest point on triangle edges
        closestPoint = p1;
        float closestDist = Vector3Distance(OriginintersectionPoint, closestPoint);
        bool closestOnEdge = false;
        int closestEdgeIdx = 0;

        Vector3 edgePoints[3] = { p1, p2, p3 };
        for (int i = 0; i < 3; i++) {
            Vector3 e1 = edgePoints[i];
            Vector3 e2 = edgePoints[(i + 1) % 3];
            bool onEdge;
            Vector3 ep = nearestPointOnLine(e1, e2, OriginintersectionPoint,&onEdge);
            float dist = Vector3Distance(OriginintersectionPoint, ep);
            if (dist < closestDist) {
                closestEdgeIdx = i;
                closestOnEdge = onEdge;
                closestDist = dist;
                closestPoint = ep;
            }
        }

        if(closestOnEdge){
            result.collisionType = EDGE;
            result.edgePoint1 = edgePoints[closestEdgeIdx];
            result.edgePoint2 = edgePoints[(closestEdgeIdx + 1) % 3];

        }else{
            result.collisionType = POINT;
            result.edgePoint1 = closestPoint;
            result.edgePoint2 = closestPoint;
        }
    }

    result.pos = closestPoint;

    return result;
}



triColEvent sphereTriCol(Vector3* Tri, Vector3 spherePos, Vector3 sphereDir, float moveDist) {

    triColEvent result;

    Vector3 p1 = Tri[0];
    Vector3 p2 = Tri[1];
    Vector3 p3 = Tri[2];

    closestPointOnTri closestPoint; 
    closestPoint = closestPointTriangle(spherePos, p1, p2, p3);

    result.closestPoint = closestPoint.pos;

    if(Vector3Distance(spherePos,closestPoint.pos) > 1.0f) {
        result.type = NONE;
        return result;
    }

    if(closestPoint.collisionType == POINT) {
        Ray pointShell = { closestPoint.pos, sphereDir };
        RayCollision shellIntersection = GetRayCollisionSphere(pointShell, spherePos, 1.0f);
        result.shellPoint = shellIntersection.point;
        Vector3 negativeSphereDir = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, sphereDir);
        Vector3 backstep = Vector3Scale(negativeSphereDir, shellIntersection.distance);
        result.newSpherePos = Vector3Add(spherePos, backstep);
        result.type = POINT;
        return result;
    }
    if (closestPoint.collisionType == EDGE) {
        result.type = EDGE;
        result.ep1 = closestPoint.edgePoint1;
        result.ep2 = closestPoint.edgePoint2;
        Vector3 edgeNormal = Vector3Normalize(Vector3Subtract(closestPoint.edgePoint2, closestPoint.edgePoint1));
        Vector3 colPlaneNormal = Vector3Normalize(Vector3Subtract(spherePos, closestPoint.pos));
        Vector3 edgeTangent = Vector3CrossProduct(edgeNormal, colPlaneNormal);

        { //Build points to display the plane, not needed in actual code, only debug
            Vector3 negativeEdgeNormal = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, edgeNormal);
            Vector3 negativeEdgeTangent = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, edgeTangent);
            result.planePoints[0] = Vector3Add(Vector3Add(closestPoint.pos, edgeNormal), negativeEdgeTangent);
            result.planePoints[1] = Vector3Add(Vector3Add(closestPoint.pos, negativeEdgeNormal), negativeEdgeTangent);
            result.planePoints[2] = Vector3Add(Vector3Add(closestPoint.pos, negativeEdgeNormal), edgeTangent);
            result.planePoints[3] = Vector3Add(Vector3Add(closestPoint.pos, edgeNormal), edgeTangent);
        }
        Ray pointShell = { closestPoint.pos, sphereDir };
        RayCollision shellIntersection = GetRayCollisionSphere(pointShell, spherePos, 1.0f);
        result.shellPoint = shellIntersection.point;
        Vector3 negativeSphereDir = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, sphereDir);

        result.colPlaneTangent = edgeTangent;

        Vector3 negativeColPlaneNormal = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, colPlaneNormal);
        Ray colPlaneRay = { closestPoint.pos, negativeColPlaneNormal };
        RayCollision shellcolPlane = GetRayCollisionSphere(colPlaneRay, spherePos, 1.0f);
        result.colPlaneShell = shellcolPlane.point;

        float colPlaneIntTime = 0.0f;
        rayPlaneIntersect(shellcolPlane.point, negativeSphereDir, closestPoint.pos, negativeColPlaneNormal, &colPlaneIntTime);

        Vector3 colPlaneIntersectionPoint = { shellcolPlane.point.x + colPlaneIntTime * negativeSphereDir.x,
                                shellcolPlane.point.y + colPlaneIntTime * negativeSphereDir.y,
                                shellcolPlane.point.z + colPlaneIntTime * negativeSphereDir.z };

        result.colPlaneIntPoint = colPlaneIntersectionPoint;

        bool onEdge;
        Vector3 ep = nearestPointOnLine(closestPoint.edgePoint1, closestPoint.edgePoint2, colPlaneIntersectionPoint, &onEdge);

        Ray newPointShell = { ep, sphereDir };
        RayCollision newShellIntersection = GetRayCollisionSphere(newPointShell, spherePos, 1.0f);

        result.newEdgePoint = ep;
        result.newShellPoint = newShellIntersection.point;

        Vector3 newbackstep = Vector3Scale(negativeSphereDir, newShellIntersection.distance);
        result.newSpherePos = Vector3Add(spherePos, newbackstep);


        return result;
    }
    if (closestPoint.collisionType == SURFACE){
        result.type = SURFACE;
        Vector3 triPlaneNormal = Vector3Normalize(Vector3Subtract(spherePos, closestPoint.pos));
        Vector3 negativeTriPlaneNormal = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, triPlaneNormal);

        Ray colRay = { closestPoint.pos, negativeTriPlaneNormal };
        RayCollision shellcol = GetRayCollisionSphere(colRay, spherePos, 1.0f);
        result.colPlaneShell = shellcol.point;

        Vector3 negativeSphereDir = Vector3Subtract((struct Vector3) { 0.0f, 0.0f, 0.0f }, sphereDir);
        float colPlaneIntTime = 0.0f;
        rayPlaneIntersect(shellcol.point, negativeSphereDir, closestPoint.pos, negativeTriPlaneNormal, &colPlaneIntTime);

        Vector3 colPlaneIntersectionPoint = { shellcol.point.x + colPlaneIntTime * negativeSphereDir.x,
                        shellcol.point.y + colPlaneIntTime * negativeSphereDir.y,
                        shellcol.point.z + colPlaneIntTime * negativeSphereDir.z };

        result.colPlaneIntPoint = colPlaneIntersectionPoint;

        closestPointOnTri newClosestPoint; 
        newClosestPoint = closestPointTriangle(colPlaneIntersectionPoint, p1, p2, p3);

        Ray newPointShell = { newClosestPoint.pos, sphereDir };
        RayCollision newShellIntersection = GetRayCollisionSphere(newPointShell, spherePos, 1.0f);

        result.newTriClosestPoint = newClosestPoint.pos;
        result.newShellPoint = newShellIntersection.point;

        Vector3 newbackstep = Vector3Scale(negativeSphereDir, newShellIntersection.distance);
        result.newSpherePos = Vector3Add(spherePos, newbackstep);

        return result;
    }

    result.type = INVALID;
    return result;
}


typedef struct State{
    Camera camera;
    Color mainSphereColor;
    Color newSphereColor;
    Vector3* ColTri;
} State;

void UpdateDrawFrame(void* v_state) {

    State* state = (State*)v_state;

    Vector3 sphere_pos = { 0.0f, 0.0f, 0.0f };
    Vector3 spere_dir = { 0.0f, 0.0f, 1.0f };
    Vector3 plane_origin = state->ColTri[0];
    Vector3 plane_normal = getTriangleNormal(state->ColTri[0],state->ColTri[1],state->ColTri[2]);

    Vector3 centerColPoint = rayPlaneIntersect(sphere_pos, spere_dir, plane_origin, plane_normal);

    Vector3 newSpherePos = centerColPoint;

    triColEvent Col = sphereTriCol(state->ColTri, newSpherePos, spere_dir, 1.0f);
    UpdateCamera(&state->camera, CAMERA_THIRD_PERSON);

    Vector3 ColTri[3] = { state->ColTri[0],state->ColTri[1],state->ColTri[2] };

    float col_length;
    
    BeginDrawing();

        ClearBackground(LIGHTGRAY);
        BeginMode3D(state->camera);

            DrawGrid(10, 1.0f);

            if (Col.type == POINT) {
                rlEnableDepthTest();
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, (struct Vector3) { 0.0f, 0.0f, 1.0f }, 0.01f, GREEN);
                DrawArrow(Col.closestPoint, Col.shellPoint, 0.01f, PURPLE);
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, Col.newSpherePos, 0.01f, PURPLE);
                DrawTriangle3D(ColTri[0], ColTri[1], ColTri[2], GRAY);
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 0.01f, RED);
                DrawSphere(Col.newSpherePos, 0.01f, RED);
                DrawSphere(Col.closestPoint, 0.01f, ORANGE);
                DrawSphere(Col.shellPoint, 0.01f, PURPLE);

                rlDrawRenderBatchActive();
                rlDisableDepthTest();
                DrawSphere(Col.newSpherePos, 1.0f, state->newSphereColor);
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, state->mainSphereColor);
            }
            if (Col.type == EDGE) {

                rlEnableDepthTest();

                //sphere movement Vector
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, (struct Vector3) { 0.0f, 0.0f, 1.0f }, 0.01f, GREEN);
                //collision plane
                DrawTriangle3D(ColTri[0], ColTri[1], ColTri[2], GRAY);
                //sphere origin
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 0.01f, RED);
                //initial closest point
                DrawSphere(Col.closestPoint, 0.02f, ORANGE);

                //vector through closest point to shell of sphere, also shows the collision plane normal
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, Col.colPlaneShell, 0.01f, ORANGE);
                //edge normal
                DrawArrow(Col.ep1, Col.ep2, 0.01f, PURPLE);
                //plane tangent
                DrawArrow(Col.closestPoint, Vector3Add(Col.closestPoint,Col.colPlaneTangent), 0.01f, RED);

                //collision plane intersection
                DrawArrow(Col.colPlaneShell, Col.colPlaneIntPoint, 0.01f, ORANGE);
                DrawSphere(Col.colPlaneIntPoint, 0.01f, GREEN);

                //new tri closest point to shell
                DrawArrow(Col.newEdgePoint, Col.newShellPoint, 0.01f, PURPLE);
                DrawSphere(Col.newEdgePoint, 0.01f, PINK);

                rlDrawRenderBatchActive();
                rlDisableDepthTest();

                DrawTriangle3D(Col.planePoints[0], Col.planePoints[1], Col.planePoints[2], Fade(YELLOW, 0.3f));
                DrawTriangle3D(Col.planePoints[2], Col.planePoints[3], Col.planePoints[0], Fade(YELLOW, 0.3f));
                DrawSphere(Col.newSpherePos, 1.0f, state->newSphereColor);
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, state->mainSphereColor);

                col_length = Vector3Distance(Col.newSpherePos,Col.newEdgePoint);

                
                

            }
            if (Col.type == SURFACE){
                rlEnableDepthTest();

                //sphere movement Vector
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, (struct Vector3) { 0.0f, 0.0f, 1.0f }, 0.01f, GREEN);
                //collision plane
                DrawTriangle3D(ColTri[0], ColTri[1], ColTri[2], GRAY);
                //sphere origin
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 0.01f, RED);
                //initial closest point
                DrawSphere(Col.closestPoint, 0.02f, ORANGE);

                //vector through closest point to shell of sphere, also shows the collision plane normal
                DrawArrow((struct Vector3) { 0.0f, 0.0f, 0.0f }, Col.colPlaneShell, 0.01f, ORANGE);

                //vector from sphere shell point back to the collision plane
                DrawArrow(Col.colPlaneShell, Col.colPlaneIntPoint, 0.01f, ORANGE);
                //point on collision plane that is intersected
                DrawSphere(Col.colPlaneIntPoint, 0.01f, GREEN);

                //if the point on the collision plane is not inside the triangle still, then we find the new point and mark it
                DrawSphere(Col.newTriClosestPoint, 0.01f, PINK);

                //vector from new triangle collison point to sphere shell in sphere movement dir
                DrawArrow(Col.newTriClosestPoint, Col.newShellPoint, 0.01f, PURPLE);

                //backstep new sphere position by the distance of the new tri col point to sphere shell
                DrawSphere(Col.newSpherePos, 1.0f, state->newSphereColor);
                DrawSphere((struct Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, state->mainSphereColor);

            }

        
        //DrawTriangle3D(v1, v2, v3, RED);
        EndMode3D();

        DrawText(TextFormat("New Sphere Pos Dist to Col Point: %f", col_length), 80, 80, 20, RED);

        //DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);

    EndDrawing();
  
}

int main(void)
{

   // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1920;
    const int screenHeight = 1080;

    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

    State state;
    Camera tmp_camera = { { 0.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    state.camera = tmp_camera;

    state.mainSphereColor = Fade(BLUE, 0.3f);
    state.newSphereColor = Fade(PURPLE, 0.3f);

    //POINT
    //Vector3 ColTri[3] = { { -0.091649f, -0.023814f, 0.851113f },{ 0.497118f, 0.976186f, 1.65942f },{ 0.497118f, -0.023814f, 1.65942f } };


    //EDGE
    //Vector3 ColTri[3] = { { -0.091649f, 1.29442f, 0.310159f },{ 0.497118f, 0.976186f, 1.65942f },{ 0.497118f, -0.579771f, 0.897624f } };

    //EDGE HARD
    Vector3 tmpColTri[3] = { { -0.681478f, -0.158351f, 0.287404f },{ 0.254617f, 0.976186f, 2.51794f },{ 0.859759f, 0.335812f, 1.83313f } };

    state.ColTri = tmpColTri;

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    
    #if defined(PLATFORM_WEB)
        emscripten_set_main_loop_arg(UpdateDrawFrame, &state, 60, 1);
    #else 
        SetTargetFPS(60); 
    // Main game loop
    while (!WindowShouldClose()) {
        UpdateDrawFrame(&state);
    }
    #endif
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

