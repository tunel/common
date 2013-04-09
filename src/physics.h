/*------------------------------------------------------------------------------
    Tune Land - Sandbox RPG
    Copyright (C) 2012-2013
        Antony Martin <antony(dot)martin(at)scengine(dot)org>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 -----------------------------------------------------------------------------*/

#ifndef H_PHYSICS
#define H_PHYSICS

#include <pthread.h>
#include <SCE/utils/SCEUtils.h>
#include <SCE/core/SCECore.h>
#ifdef PHY_GET_MESH
#include <SCE/interface/SCEInterface.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "position.h"

/* default world size (in case of aabb) */
#define PHY_WORLD_SIZE 8000.0
/* default grid size for the world */
#define PHY_GRID_WIDTH 8000
#define PHY_GRID_HEIGHT 8000
#define PHY_GRID_DEPTH 8000
/* maximum sub-steps on step simulation */
#define PHY_MAX_SUBSTEPS 10

/* options for copying */
#if 0
/* suxor */
#define PHY_COPY_ABS_POINT1 1
#define PHY_COPY_REL_POINT1 (1<<1)
#define PHY_COPY_DEPTH1 (1<<2)
#define PHY_COPY_ABS_POINT2 (1<<3)
#define PHY_COPY_REL_POINT2 (1<<4)
#define PHY_COPY_DEPTH2 (1<<5)
#define PHY_COPY_NORMAL_1TO2 (1<<6)
#define PHY_COPY_EVERYTHING (~0)
#else
#define PHY_COPY_POINT1 1
#define PHY_COPY_POINT2 (1<<1)
#endif

typedef enum {
    PHY_WORLD_AABB,
    PHY_WORLD_DBVT,
    PHY_WORLD_GPUGRID
} PhyWorldType;

typedef enum {
    PHY_NONE_BODY = 0,
    PHY_DYNAMIC_BODY,
    PHY_STATIC_BODY,
    PHY_KINEMATIC_BODY
} PhyBodyType;


typedef struct physicstrimesh PhysicsTriMesh;
struct physicstrimesh {
    SCE_SGeometry *geometry;    /* SCE geometry */
    int canfree_geom;
    void *trimesh;              /* btTriangleIndexVertexArray */
    int *indices;
    int convex;
    float x, y, z;              /* surfaces (used to compute fluid friction) */
};

typedef struct physicsshape PhysicsShape;
struct physicsshape {
    int canfree_parent;
    PhysicsShape *parent;
    void *shape;                /* btCollisionShape */
    void *scaledbvh;            /* btScaledBvhTriangleMeshShape */
    int dynamic;                /* GImpactMeshShape or BvhTriangleMeshShape? */
    float x, y, z;              /* surfaces (used to compute fluid friction) */
    PhysicsTriMesh *trimesh;
    int canfree;
    SCE_TMatrix4 matrix;
    SCE_SListIterator it;
};

typedef struct physicsshapes PhysicsShapes;
struct physicsshapes {
    void *shape;                /* final shape, btCompoundShape if many */
    float x, y, z;              /* total size */
    SCE_SList shapes;           /* PhysicsShape */
    int built;
};

typedef struct phyworld PhyWorld;
typedef struct physics Physics;

typedef void (*PhyUpdateFunc)(Physics*, SCE_TMatrix4);

struct physics {
    PhyBodyType type;
    int typeobj;                /* object type */
    short typemask;             /* mask */
    short collide_with;         /* mask */
    float mass;
    float friction;
    void *body;                 /* btRigidBody */
    PhysicsShapes *shapes;
    int canfree_shapes;

    void *motionstate;          /* PhyMotionState */

    PhyUpdateFunc updatefunc;   /* update position callback */

    /* position relative informations */
    SCE_TMatrix4 mat;
    SCE_TMatrix4 inv_mat;
    int updated;                /* does the inverse matrix have been updated? */
    /* * */

    pthread_mutex_t *mutex;
    int is_static;              /* deprecated... ? */
    void *userdata;
    float rho;
    int use_rho;
    PhyWorld *world;
    SCE_SListIterator it;
};


typedef struct phycharacter PhyCharacter;
struct phycharacter {
    Physics *phy;
    SCE_TVector3 target_vel;
    void *action;
    float radius;
    float height;
    int on_ground;
};

/* lol */
typedef enum {
    PHY_NO = SCE_FALSE,
    PHY_YES = SCE_TRUE,
    PHY_MAYBE
} PhyColState;

typedef struct phycollision PhyCollision;
struct phycollision {
    void *pair;
    void *dispatcher;
    void *dispatcherinfo;
    int done;
    PhyColState need;
    int inverse;
};

/* collision callback prototype */
typedef void (*PhyCollisionCallbackFunc)(PhyWorld*, Physics*, Physics*,
                                         PhyCollision*, void*);

typedef struct phycollisionpoint PhyCollisionPoint;
struct phycollisionpoint {
    SCE_TVector3 abs_pos;
    SCE_TVector3 rel_pos;
};

typedef struct phycollisioninfo PhyCollisionInfo;
struct phycollisioninfo {
    PhyCollisionPoint *p1, *p2;
    PhyCollisionPoint point1, point2;
    SCE_TVector3 normal;
    float distance;
};


struct phyworld {
    void *colconf;              /* btDefaultCollisionConfiguration */
    void *disp;                 /* btCollisionDispatcher */
    void *cs;                   /* btSequentialImpulseConstraintSolver */
    void *paircache;            /* btBroadphaseInterface */
    void *world;                /* btDiscreteDynamicsWorld */

    PhyCollisionCallbackFunc near_callback;
    void *near_data;
    void *current_pair;         /* btBroadphasePair */
    void *manifold_array;       /* btManifoldArray */

    SCE_SList fluid_bodies;     /* bodies using fluid friction */
};

int Init_Phy (void);
void Quit_Phy (void);

PhyWorld* Phy_NewWorld (PhyWorldType);
void Phy_FreeWorld (PhyWorld*);
void Phy_SetupDebug (PhyWorld*);

void Phy_SetGravity (PhyWorld*, float, float, float);
void Phy_SetGravityv (PhyWorld*, SCE_TVector3);

PhysicsTriMesh* Phy_NewTriMesh (SCE_SGeometry*, int, int);
void Phy_FreeTriMesh (PhysicsTriMesh*);

PhysicsShape* Phy_NewSphereShape (float);
PhysicsShape* Phy_NewBoxShape (float, float, float);
PhysicsShape* Phy_NewBoxShapev (SCE_TVector3);
PhysicsShape* Phy_NewCapsuleShape (float, float);
PhysicsShape* Phy_NewTriMeshShape (int, PhysicsTriMesh*, int, int);
PhysicsShape* Phy_NewVoxelTerrainShape (SCE_SVoxelWorld*, float, int);
void Phy_FreeShape (PhysicsShape*);

PhysicsShapes* Phy_NewShapes (void);
PhysicsShapes* Phy_NewShapesFromShape (const SCE_TMatrix4, PhysicsShape*);
void Phy_FreeShapes (PhysicsShapes*);

void Phy_DefaultUpdateFunc (Physics*, SCE_TMatrix4);

Physics* Phy_New (PhyBodyType);
void Phy_Free (Physics*);

void Phy_AddShape (PhysicsShapes*, const SCE_TMatrix4, PhysicsShape*);
int Phy_BuildShapes (PhysicsShapes*);

PhyBodyType Phy_GetType (Physics*);

void Phy_SetObjType (Physics*, int);
int Phy_GetObjType (Physics*);
void Phy_SetTypeMask (Physics*, short);
short Phy_GetTypeMask (Physics*);

void Phy_CollideWith (Physics*, short);
void Phy_AddCollide (Physics*, int);
void Phy_RemoveCollide (Physics*, int);
void Phy_CollideWithEverything (Physics*);

void Phy_SetData (Physics*, void*);
void* Phy_GetData (Physics*);

void Phy_SetMass (Physics*, float);
float Phy_GetMass (Physics*);

void Phy_SetFriction (Physics*, float);
float Phy_GetFriction (Physics*);

void Phy_SetUpdateFunction (Physics*, PhyUpdateFunc);

void Phy_GetMatrix4v (Physics*, SCE_TMatrix4);
void Phy_GetInvMatrix4v (Physics*, SCE_TMatrix4);
void Phy_GetPos (Physics*, Position*);

void Phy_SetMutex (Physics*, pthread_mutex_t*);

void Phy_SetShapes (Physics*, PhysicsShapes*);
PhysicsShapes* Phy_GetShapes (Physics*);
#ifdef PHY_GET_MESH
SCE_SMesh* Phy_GetMesh (Physics*);
#endif

void Phy_Activate (Physics*, int);
int Phy_IsActivated (Physics*);
void Phy_SetPersistentActivation (Physics*, int);

int Phy_IsStatic (Physics*);

/* those functions are just set to initialize a physics body
   or move a kinematic body */
void Phy_SetPosition (Physics*, float, float, float);
void Phy_SetPositionv (Physics*, const SCE_TVector3);
#if 0
void Phy_SetRotation (Physics*, float, float, float);
void Phy_SetRotationv (Physics*, SCE_TVector3);
#endif
void Phy_SetOrientation (Physics*, SCE_TQuaternion);
void Phy_SetMatrix (Physics*, SCE_TMatrix4);

int Phy_Instanciate (Physics*, Physics*);
Physics* Phy_NewInstanciate (Physics*, PhyBodyType);

int Phy_Build (Physics*);

void Phy_Add (PhyWorld*, Physics*);
void Phy_Remove (Physics*);

void Phy_SetLinearVelAbs (Physics*, float, float, float);
void Phy_SetLinearVelAbsv (Physics*, const SCE_TVector3);
void Phy_SetAngularVelAbs (Physics*, float, float, float);
void Phy_SetAngularVelAbsv (Physics*, const SCE_TVector3);

void Phy_SetLinearVelRel (Physics*, float, float, float);
void Phy_SetLinearVelRelv (Physics*, const SCE_TVector3);
void Phy_SetAngularVelRel (Physics*, float, float, float);
void Phy_SetAngularVelRelv (Physics*, const SCE_TVector3);


void Phy_SetLinearImpulseAbs (Physics*, float, float, float);
void Phy_SetLinearImpulseAbsv (Physics*, const SCE_TVector3);
void Phy_SetAngularImpulseAbs (Physics*, float, float, float);
void Phy_SetAngularImpulseAbsv (Physics*, const SCE_TVector3);

void Phy_SetLinearImpulseRel (Physics*, float, float, float);
void Phy_SetLinearImpulseRelv (Physics*, const SCE_TVector3);
void Phy_SetAngularImpulseRel (Physics*, float, float, float);
void Phy_SetAngularImpulseRelv (Physics*, const SCE_TVector3);


void Phy_GetLinearVelocityAbsv (Physics*, SCE_TVector3);
void Phy_GetAngularVelocityAbsv (Physics*, SCE_TVector3);
void Phy_GetLinearVelocityRelv (Physics*, SCE_TVector3);
void Phy_GetAngularVelocityRelv (Physics*, SCE_TVector3);

/* fluid friction: F = -rho/2 * S * v * ~secs */
void Phy_SetRho (Physics*, float);

void Phy_UpdateWorld (PhyWorld*, float);

void Phy_SetCollisionCallback (PhyWorld*, PhyCollisionCallbackFunc, void*);

int Phy_NeedsCollision (PhyCollision*);
void Phy_ProcessCollision (PhyCollision*);
int Phy_GetCollisionInfov (PhyCollision*, unsigned int, PhyCollisionInfo*, int);

PhyCharacter* Phy_NewCharacter (void);
void Phy_FreeCharacter (PhyCharacter*);

void Phy_SetCharacterDimensions (PhyCharacter*, float, float);
void Phy_SetCharacterPosition (PhyCharacter*, float, float, float);
void Phy_SetCharacterPositionv (PhyCharacter*, const SCE_TVector3);
void Phy_GetCharacterPos (PhyCharacter*, Position*);

int Phy_BuildCharacter (PhyCharacter*);

void Phy_AddCharacter (PhyWorld*, PhyCharacter*);
void Phy_RemoveCharacter (PhyCharacter*);

void Phy_SetCharacterVelocity (PhyCharacter*, float, float, float);
void Phy_SetCharacterVelocityv (PhyCharacter*, const SCE_TVector3);

int Phy_IsCharacterOnGround (const PhyCharacter*);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* guard */
