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

/*#include <iostream>*/
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#ifdef PHY_GET_MESH
#include <SCE/interface/SCEInterface.h>
#endif

#ifdef TL_PHY_DEBUG
#include <GL/gl.h>
#endif
#include "physics.h"

static const char *const cppallocfailmsg = "C++ cannot allocate memory hahaha!";

static int resource_trimesh_type = 0;
static int resource_staticshape_type = 0;
static int resource_shapes_type = 0;
/* a physical object isn't considered as a resource, it's just using resources
   but each physical object is an independant object in the game world */
/*static int resource_type = 0;*/

#define cppallocfail do {                               \
        SCEE_Log (SCE_OUT_OF_MEMORY);                   \
        SCEE_LogMsg (cppallocfailmsg); } while (0)

static inline void Phy_CopyVector3 (SCE_TVector3 v, const btVector3& bv)
{
    v[0] = bv.getX (); v[1] = bv.getY (); v[2] = bv.getZ ();
}

static void Phy_MatrixFromTransform (SCE_TMatrix4 m, const btTransform& t)
{
    btScalar mat[16];
    t.getOpenGLMatrix (mat);
    // TODO: check if a macro defines the type of btScalar
    for (int i = 0; i < 16; i++)
        m[i] = mat[i];
    SCE_Matrix4_TransposeCopy (m);
}
static void Phy_TransformFromMatrix (btTransform& t, const SCE_TMatrix4 m)
{
    SCE_TMatrix4 matrix;
    btScalar mat[16];
    SCE_Matrix4_Identity (matrix);
    SCE_Matrix4_Transpose (m, matrix);
    // TODO: check if a macro defines the type of btScalar
    for (int i = 0; i < 16; i++)
        mat[i] = matrix[i];
    t.setFromOpenGLMatrix (mat);
}

class PhyMotionState: public btMotionState {
protected:
    btTransform m_trans;
public:
    Physics *m_phy;
    PhyMotionState (const btTransform &initialpos = btTransform::getIdentity()):
        m_phy(NULL), m_trans(initialpos) {}
    ~PhyMotionState () {}
    virtual void getWorldTransform (btTransform &worldTrans) const {
        worldTrans = m_trans;
    }
    void setWorldTransform (const btTransform& trans) {
        SCE_TMatrix4 mat;
        m_trans = trans;
        Phy_MatrixFromTransform (mat, trans);
        m_phy->updated = SCE_FALSE;
        if (!m_phy->mutex) {
            m_phy->updatefunc (m_phy, mat);
        } else {
            if (!pthread_mutex_lock (m_phy->mutex)) {
                m_phy->updatefunc (m_phy, mat);
                pthread_mutex_unlock (m_phy->mutex);
            }
        }
    }
};

class PhyTerrainShape: public btConcaveShape {
private:
    mutable SCE_SVoxelWorld *vw;
    float voxel_unit;
    int width;

    mutable SCE_SGrid grid;
    mutable SCE_SMCGenerator mc;
    mutable size_t n_vertices;
    mutable size_t n_indices;
    mutable SCEvertices *vertices;
    mutable SCEindices *indices;

#ifdef PHY_GET_MESH
    SCE_SGeometry *geom;
    SCE_SMesh *mesh;
#endif

    btVector3 m_scaling;

public:

    PhyTerrainShape (SCE_SVoxelWorld *world, float unit = 1.0, int w = 5) :
        vw (world),
        voxel_unit (unit),
        width (w),
        n_vertices (0),
        n_indices (0),
        vertices (NULL),
        indices (NULL),
#ifdef PHY_GET_MESH
        geom (NULL),
        mesh (NULL),
#endif
        m_scaling (1.0, 1.0, 1.0) {
        size_t n;

        m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE;

        n_indices = 0;
        n_vertices = 0;

        SCE_Grid_Init (&grid);
        SCE_Grid_SetDimensions (&grid, width, width, width);
        SCE_Grid_SetPointSize (&grid, 1);
        if (SCE_Grid_Build (&grid) < 0) goto fail;

        SCE_MC_Init (&mc);
        n = SCE_Grid_GetNumPoints (&grid);
        SCE_MC_SetNumCells (&mc, n);
        if (SCE_MC_Build (&mc) < 0) goto fail;

        if (!(vertices = (SCEvertices*)SCE_malloc (9 * n * sizeof *vertices)))
            goto fail;
        if (!(indices = (SCEindices*)SCE_malloc (15 * n * sizeof *indices)))
            goto fail;

        return;
    fail:
        // TODO: well... throw something?
        return;
    }

    ~PhyTerrainShape () {
        // TODO: not yet called
        SCE_Grid_Clear (&grid);
        SCE_MC_Clear (&mc);
        SCE_free (vertices);
        SCE_free (indices);
    }

    virtual int getShapeType () const {
        // return TERRAIN_SHAPE_PROXYTYPE;
        return CUSTOM_CONCAVE_SHAPE_TYPE;
    }

    virtual const char* getName () const {
        return "VOXEL_TERRAIN";
    }

    virtual void getAabb (const btTransform &t, btVector3 &aabbMin,
                          btVector3 &aabbMax) const {
        // return something big (like really big).
        btScalar big = 1000000000.0;
        aabbMin.setValue (-big, -big, -big);
        aabbMax.setValue (big, big, big);
    }

    virtual void calculateLocalInertia (btScalar mass, btVector3 &inertia) const {
        inertia = btVector3 (0.0, 0.0, 0.0);
    }

    virtual void setLocalScaling (const btVector3 &scaling) {
        // why should I need that?
        m_scaling = scaling;
    }

    virtual const btVector3& getLocalScaling () const {
        return m_scaling;
    }

    virtual void processAllTriangles (btTriangleCallback *callback,
                                      const btVector3 &inf,
                                      const btVector3 &sup) const {
        SCE_SLongRect3 rect;
        SCE_SIntRect3 area;
        size_t i;
        long w, h, d;
        float u = voxel_unit;

        // some random offsets to ensure we grab enough voxels
        // -1 because ??
        // +1 for ceil(), +1 because area->w = w - 1, +1 because.. ??
        SCE_Rectangle3_Setl (&rect,
                             inf.x() / u - 1, inf.y() / u - 1, inf.z() / u - 1,
                             sup.x() / u + 3, sup.y() / u + 3, sup.z() / u + 3);

        w = SCE_Rectangle3_GetWidthl (&rect);
        h = SCE_Rectangle3_GetHeightl (&rect);
        d = SCE_Rectangle3_GetDepthl (&rect);

        if (w * h * d > width * width * width) {
            SCEE_SendMsg ("processAllTriangles(): region is too big\n");
            return;
        }

        SCE_Grid_FillupZeros (&grid);
        // and we sure hope this call works fine: region exists, doesnt
        // touch border, and so forth.
        SCE_VWorld_GetRegion (vw, 0, &rect, (SCEubyte*)SCE_Grid_GetRaw (&grid));

        SCE_Grid_SetDimensions (&grid, w, h, d);
        SCE_Rectangle3_Set (&area, 0, 0, 0, w - 1, h - 1, d - 1);

        n_vertices = SCE_MC_GenerateVertices (&mc, &area, &grid, vertices);
        n_indices = SCE_MC_GenerateIndices (&mc, indices);

        // transform vertices into world space
        long p1[3], p2[3];
        SCE_Rectangle3_GetPointslv (&rect, p1, p2);
        for (i = 0; i < n_vertices; i++) {
            vertices[i * 3 + 0] *= w;
            vertices[i * 3 + 1] *= h;
            vertices[i * 3 + 2] *= d;
            vertices[i * 3 + 0] = u * (vertices[i * 3 + 0] + p1[0]);
            vertices[i * 3 + 1] = u * (vertices[i * 3 + 1] + p1[1]);
            vertices[i * 3 + 2] = u * (vertices[i * 3 + 2] + p1[2]);
        }

        // process triangles
        SCEvertices *v = vertices;
        SCEindices *ind = indices;
        size_t tri_id = 0;
        for (i = 0; i < n_indices; tri_id++) {
            btVector3 p[3];
            p[0].setValue (v[ind[i] * 3], v[ind[i] * 3 + 1], v[ind[i] * 3 + 2]);
            i++;
            p[1].setValue (v[ind[i] * 3], v[ind[i] * 3 + 1], v[ind[i] * 3 + 2]);
            i++;
            p[2].setValue (v[ind[i] * 3], v[ind[i] * 3 + 1], v[ind[i] * 3 + 2]);
            i++;
            callback->processTriangle (p, 0, tri_id);
        }
    }

#ifdef PHY_GET_MESH
    SCE_SMesh* getMesh () {

        if (!mesh) {
            SCE_SGeometryArray ar1, ar2;

            geom = SCE_Geometry_Create ();

            SCE_Geometry_InitArray (&ar1);
            SCE_Geometry_SetArrayData (&ar1, SCE_POSITION, SCE_VERTICES_TYPE, 0, 3,
                                       NULL, SCE_FALSE);
            SCE_Geometry_AddArrayDup (geom, &ar1, SCE_FALSE);
            SCE_Geometry_InitArray (&ar1);
            SCE_Geometry_SetArrayIndices (&ar1, SCE_INDICES_TYPE, NULL, SCE_FALSE);
            SCE_Geometry_SetIndexArrayDup (geom, &ar1, SCE_FALSE);
            SCE_Geometry_SetPrimitiveType (geom, SCE_TRIANGLES);

            SCE_Geometry_SetNumVertices (geom, width * width * width * 3);
            SCE_Geometry_SetNumIndices (geom, width * width * width * 15);

            mesh = SCE_Mesh_CreateFrom (geom, 1);
            SCE_Mesh_AutoBuild (mesh);
        }

        size_t size;

        SCE_Mesh_SetNumVertices (mesh, n_vertices);
        SCE_Mesh_SetNumIndices (mesh, n_indices);

        size = n_vertices * 3 * sizeof (SCEvertices);
        SCE_Mesh_UploadVertices (mesh, SCE_MESH_STREAM_G, vertices, 0, size);
        size = n_indices * sizeof (SCEindices);
        SCE_Mesh_UploadIndices (mesh, indices, size);

        return mesh;
    }
#endif
};

static void* Phy_LoadTriMeshResource (const char*, int, void*);
static void* Phy_LoadStaticTriMeshShapeResource (const char*, int, void*);
static void* Phy_LoadShapesResource (const char*, int, void*);
/*static void* Phy_LoadResource (const char*, int, void*);*/

int Init_Phy (void)
{
    // register loaders and resource types
    resource_trimesh_type = SCE_Resource_RegisterType (
        SCE_FALSE, Phy_LoadTriMeshResource, NULL);
    resource_staticshape_type = SCE_Resource_RegisterType
        (SCE_FALSE, Phy_LoadStaticTriMeshShapeResource, NULL);
#if 0
    resource_type = SCE_Resource_RegisterType (
        SCE_FALSE, Phy_LoadResource, NULL);
#endif

#if 0
    SCE_Media_Register (resource_trimesh_type, ".conf",
                        Phy_LoadTriMeshResource, NULL);
    SCE_Media_Register (resource_shapes_type, ".conf",
                        Phy_LoadShapesResource, NULL);
    SCE_Media_Register (resource_type, ".conf", Phy_LoadResource, NULL);
#endif
    // TODO: check returned values lolz

    return SCE_OK;
}
void Quit_Phy (void)
{
}



#ifdef TL_PHY_DEBUG
class GLDebugDrawer : public btIDebugDraw
{
    int m_debugMode;

public:

    GLDebugDrawer ();
            
    virtual void drawLine (const btVector3& from, const btVector3& to,
                           const btVector3& color);
      
    virtual void drawContactPoint(const btVector3& PointOnB,
                                  const btVector3& normalOnB,
                                  btScalar distance, int lifeTime,
                                  const btVector3& color);
      
    virtual void reportErrorWarning (const char* warningString);
      
    virtual void draw3dText (const btVector3& location, const char* textString);
      
    virtual void setDebugMode (int debugMode);
      
    virtual int getDebugMode () const { return m_debugMode; }
      
};

GLDebugDrawer::GLDebugDrawer ()
    :m_debugMode(0)
{
   
}

void GLDebugDrawer::drawLine (const btVector3& from, const btVector3& to,
                              const btVector3& color)
{
    glBegin (GL_LINES);
    glColor4f (color.getX(), color.getY(), color.getZ(), 1.0f);
    glVertex3f (from.getX(), from.getY(), from.getZ());
    glVertex3f (to.getX(), to.getY(), to.getZ());
    glEnd ();
}

void GLDebugDrawer::setDebugMode (int debugMode)
{
    m_debugMode = debugMode;
}

void GLDebugDrawer::draw3dText (const btVector3& location,
                                const char* textString)
{
    //glRasterPos3f(location.x(),  location.y(),  location.z());
    //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void GLDebugDrawer::reportErrorWarning (const char* warningString)
{
    printf (warningString);
}

void GLDebugDrawer::drawContactPoint (const btVector3& pointOnB,
                                      const btVector3& normalOnB,
                                      btScalar distance, int lifeTime,
                                      const btVector3& color)
{
    //btVector3 to=pointOnB+normalOnB*distance;
    //const btVector3&from = pointOnB;
    //glColor4f(color.getX(), color.getY(), color.getZ(), 1.0f);
      
    //GLDebugDrawer::drawLine(from, to, color);
      
    //glRasterPos3f(from.x(),  from.y(),  from.z());
    //char buf[12];
    //sprintf(buf," %d",lifeTime);
    //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
}
#endif  // TL_PHY_DEBUG



static void Phy_InitWorld (PhyWorld *world)
{
    world->colconf = NULL;
    world->disp = NULL;
    world->cs = NULL;
    world->paircache = NULL;
    world->world = NULL;

    world->near_callback = NULL;
    world->near_data = NULL;
    world->current_pair = NULL;
    world->manifold_array = NULL;

    SCE_List_Init (&world->fluid_bodies);
}
PhyWorld* Phy_NewWorld (PhyWorldType worldtype)
{
    btDefaultCollisionConfiguration *col = NULL;
    btCollisionDispatcher *dis = NULL;
    btSequentialImpulseConstraintSolver *cs = NULL;
    btBroadphaseInterface *paircache = NULL;
    btDiscreteDynamicsWorld *world = NULL;
    btManifoldArray *marray = NULL;
    PhyWorld *pworld = NULL;

    if (!(pworld = (PhyWorld*)SCE_malloc (sizeof *pworld))) goto fail;
    Phy_InitWorld (pworld);

    col = new btDefaultCollisionConfiguration ();
    if (col == NULL) goto cppfail;
    dis = new btCollisionDispatcher (col);
    if (dis == NULL) goto cppfail;
    cs = new btSequentialImpulseConstraintSolver;
    if (cs == NULL) goto cppfail;

    switch (worldtype) {
    case PHY_WORLD_DBVT: {
        paircache = new btDbvtBroadphase;
    }
        break;
#if 0
    case PHY_WORLD_GPUGRID: {
        btScalar s (PHY_WORLD_SIZE / 2);
        btVector3 minaabb (-s, -s, -s);
        btVector3 maxaabb (s, s, s);
        paircache = new btGpu3DGridBroadphase (minaabb, maxaabb, PHY_GRID_WIDTH,
                                               PHY_GRID_HEIGHT, PHY_GRID_DEPTH,
                                               10, 1000, 42);
        // TODO: what are these arbitrary values? mean 10, 1000, 42
    }
        break;
#endif
//    case PHY_WORLD_AABB:
    default:
    {
        unsigned int maxhandles = 1024;
        btScalar s (PHY_WORLD_SIZE / 2);
        btVector3 minaabb (-s, -s, -s);
        btVector3 maxaabb (s, s, s);
        paircache = new bt32BitAxisSweep3 (minaabb, maxaabb, maxhandles);
    }
    }
    if (paircache == NULL) goto cppfail;

    world = new btDiscreteDynamicsWorld (dis, paircache, cs, col);
    if (world == NULL) goto cppfail;
    world->setGravity (btVector3 (0.0, 0.0, 0.0));

    // setup gimpact collision algorithm
    btGImpactCollisionAlgorithm::registerAlgorithm (dis);

    marray = new btManifoldArray;
    if (marray == NULL) goto cppfail;
    marray->reserve (1);        // TODO: verify success of allocation

    pworld->colconf = (void*)col;
    pworld->disp = (void*)dis;
    pworld->cs = (void*)cs;
    pworld->paircache = (void*)paircache;
    pworld->world = (void*)world;

    pworld->manifold_array = (void*)marray;

    return pworld;
fail:
    Phy_FreeWorld (pworld);
    SCEE_LogSrc ();
    return NULL;
cppfail:
    Phy_FreeWorld (pworld);
    cppallocfail;
    return NULL;
}
void Phy_SetupDebug (PhyWorld *pworld)
{
#ifdef TL_PHY_DEBUG
    btDiscreteDynamicsWorld *world = (btDiscreteDynamicsWorld*)pworld->world;
    world->setDebugDrawer (new GLDebugDrawer);
    world->getDebugDrawer()->setDebugMode (btIDebugDraw::DBG_DrawWireframe);
#endif
}
void Phy_FreeWorld (PhyWorld *pworld)
{
    if (pworld) {
        btDefaultCollisionConfiguration *col = NULL;
        btCollisionDispatcher *dis = NULL;
        btSequentialImpulseConstraintSolver *cs = NULL;
        btDiscreteDynamicsWorld *world = NULL;
        col = (btDefaultCollisionConfiguration*)pworld->colconf;
        dis = (btCollisionDispatcher*)pworld->disp;
        cs = (btSequentialImpulseConstraintSolver*)pworld->cs;
        world = (btDiscreteDynamicsWorld*)pworld->world;

        delete world;
        delete col;
        delete dis;
        delete cs;

        SCE_List_Clear (&pworld->fluid_bodies);
        SCE_free (pworld);
    }
}


void Phy_SetGravity (PhyWorld *world, float x, float y, float z)
{
    btDiscreteDynamicsWorld *w = (btDiscreteDynamicsWorld*)world->world;
    w->setGravity (btVector3 (x, y, z));
}

void Phy_SetGravityv (PhyWorld *world, SCE_TVector3 v)
{
    btDiscreteDynamicsWorld *w = (btDiscreteDynamicsWorld*)world->world;
    w->setGravity (btVector3 (v[0], v[1], v[2]));
}


static int Phy_BuildTriMesh (PhysicsTriMesh *trimesh)
{
    SCE_SGeometry *geom = trimesh->geometry;
#if 1
    size_t n_indices = SCE_Geometry_GetNumIndices (geom);
    int *indices = new int[SCE_Type_Sizeof (SCE_INT) * n_indices];
    if (!indices) {
        cppallocfail;
        return SCE_ERROR;
    }
    trimesh->indices = indices;
    SCE_Type_Convert (SCE_INT, indices, SCE_INDICES_TYPE,
                      SCE_Geometry_GetIndices (geom), n_indices);

    btTriangleIndexVertexArray *bttrimesh = new btTriangleIndexVertexArray (
        SCE_Geometry_GetNumPrimitives (geom),
        indices,
        3 * sizeof (int),
        SCE_Geometry_GetNumVertices (geom),
        // TODO: hope that btScalar and SCEvertices are the same type.
        (btScalar*)SCE_Geometry_GetPositions (geom),
        3 * sizeof (SCEvertices));
    if (!bttrimesh) {
        cppallocfail;
        return SCE_ERROR;
    }
#else
    btIndexedMesh mesh;
    mesh.m_numTriangles = SCE_Geometry_GetNumPrimitives (geom);
    mesh.m_triangleIndexBase =
        (const unsigned char*)SCE_Geometry_GetIndices (geom);
    mesh.m_triangleIndexStride = 0;
    mesh.m_numVertices = SCE_Geometry_GetNumVertices (geom);
    mesh.m_vertexBase =
        (const unsigned char*)SCE_Geometry_GetPositions (geom);
    mesh.m_vertexStride = 0;    // use stride
    mesh.m_indexType = PHY_SHORT;
    mesh.m_vertexType = PHY_FLOAT;
    btTriangleIndexVertexArray *bttrimesh = new btTriangleIndexVertexArray;
    bttrimesh->addIndexedMesh (mesh, PHY_SHORT);
#endif
    trimesh->trimesh = bttrimesh;
    return SCE_OK;
}
PhysicsTriMesh* Phy_NewTriMesh (SCE_SGeometry *geom, int convex, int canfree)
{
    float w, h, d;
    SCE_SBox *box = NULL;
    PhysicsTriMesh *trimesh = (PhysicsTriMesh*)SCE_malloc (sizeof *trimesh);
    if (!trimesh)
        goto failure;
    trimesh->geometry = geom;
    trimesh->canfree_geom = canfree;
    trimesh->trimesh = NULL;
    trimesh->indices = NULL;
    trimesh->convex = convex;
    if (Phy_BuildTriMesh (trimesh) < 0)
        goto failure;
    // update geometry box
    SCE_Geometry_GenerateBoundingBox (geom);
    box = SCE_Geometry_GetBox (geom);
    // extract plane surfaces
    SCE_Box_GetDimensionsv (box, &w, &h, &d);
    // TODO: must: improve that by computing real mesh surface using
    //       occlusion queries
#define S_PROBA 0.4
    trimesh->x = h * d * S_PROBA;
    trimesh->y = w * d * S_PROBA;
    trimesh->z = w * h * S_PROBA;
#undef S_PROBA
    return trimesh;
failure:
    Phy_FreeTriMesh (trimesh);
    SCEE_LogSrc ();
    return NULL;
}
void Phy_FreeTriMesh (PhysicsTriMesh *trimesh)
{
    if (trimesh) {
        if (SCE_Resource_Free (trimesh)) {
            btTriangleIndexVertexArray *bttrimesh =
                (btTriangleIndexVertexArray*)trimesh->trimesh;
            delete bttrimesh;
            if (trimesh->canfree_geom)
                SCE_Geometry_Delete (trimesh->geometry);
            // indices are owned by Bullet
            SCE_free (trimesh);
        }
    }
}


static void Phy_InitShape (PhysicsShape *shape)
{
    shape->canfree_parent = SCE_FALSE;
    shape->parent = NULL;
    shape->shape = NULL;
    shape->scaledbvh = NULL;
    shape->dynamic = SCE_FALSE;
    shape->x = shape->y = shape->z = 0.0;
    shape->trimesh = NULL;
    shape->canfree = SCE_FALSE;
    SCE_Matrix4_Identity (shape->matrix);
    SCE_List_InitIt (&shape->it);
    SCE_List_SetData (&shape->it, shape);
}
static void Phy_ClearShape (PhysicsShape *shape)
{
    // Phy_NewTriMeshShapeCopy() makes it NULL
    if (shape->shape) {
        btCollisionShape *colshape = (btCollisionShape*)shape->shape;
        delete colshape;
    }
    if (shape->canfree)
        Phy_FreeTriMesh (shape->trimesh);
    SCE_List_Remove (&shape->it);
    if (shape->scaledbvh) {
        btScaledBvhTriangleMeshShape *s = NULL;
        s = (btScaledBvhTriangleMeshShape*)shape->scaledbvh;
        delete s;
    }
    if (shape->canfree_parent)
        Phy_FreeShape (shape->parent);
}

// TODO: add NewSphereSphereShape(radius, scale) function (btSphereSphere)
PhysicsShape* Phy_NewSphereShape (float radius)
{
    btCollisionShape *colshape;
    PhysicsShape *shape = (PhysicsShape*)SCE_malloc (sizeof *shape);
    if (!shape)
        goto failure;
    Phy_InitShape (shape);
    colshape = new btSphereShape (btScalar (radius));
    if (colshape == NULL) {
        cppallocfail;
        goto failure;
    }
    shape->shape = colshape;
    shape->x = shape->y = shape->z = M_PI * radius * radius;

    return shape;
failure:
    Phy_FreeShape (shape);
    SCEE_LogSrc ();
    return NULL;
}
PhysicsShape* Phy_NewBoxShape (float w, float h, float d)
{
    btCollisionShape *colshape;
    PhysicsShape *shape = (PhysicsShape*)SCE_malloc (sizeof *shape);
    if (!shape)
        goto failure;
    Phy_InitShape (shape);
    colshape = new btBoxShape (btVector3 (w, h, d));
    if (colshape == NULL) {
        cppallocfail;
        goto failure;
    }
    shape->shape = colshape;
    shape->x = h * d;
    shape->y = w * d;
    shape->z = w * h;

    return shape;
failure:
    Phy_FreeShape (shape);
    SCEE_LogSrc ();
    return NULL;
}
PhysicsShape* Phy_NewBoxShapev (SCE_TVector3 v)
{
    return Phy_NewBoxShape (v[0], v[1], v[2]);
}
PhysicsShape* Phy_NewTriMeshShape (int dynamic, PhysicsTriMesh *trimesh,
                                   int scale, int canfree)
{
    btCollisionShape *colshape = NULL;
//    btGImpactMeshShape *colshape = NULL;
    btTriangleIndexVertexArray *bttrimesh;
    PhysicsShape *shape = (PhysicsShape*)SCE_malloc (sizeof *shape);
    if (!shape)
        goto failure;
    Phy_InitShape (shape);
    bttrimesh = (btTriangleIndexVertexArray*)trimesh->trimesh;
    if (dynamic) {
        btGImpactMeshShape *gishape = new btGImpactMeshShape (bttrimesh);
        gishape->updateBound ();   // wat dat?
        colshape = gishape;
    } else {
        if (trimesh->convex)        // use btConvexHullShape instead... ?
            colshape = new btConvexTriangleMeshShape (bttrimesh);
        else {
            btBvhTriangleMeshShape *thebvh = NULL;
            thebvh = new btBvhTriangleMeshShape (bttrimesh, true);
            // TODO: never called in practice.
            if (scale) {
                btScaledBvhTriangleMeshShape *scb = NULL;
                scb = new btScaledBvhTriangleMeshShape (thebvh,
                                                        btVector3 (1.0, 1.0, 1.0));
                shape->scaledbvh = (void*)scb;
            }
            colshape = thebvh;
        }
    }
    if (colshape == NULL) {
        cppallocfail;
        goto failure;
    }
    shape->shape = colshape;
    shape->dynamic = dynamic;
    shape->x = trimesh->x;
    shape->y = trimesh->y;
    shape->z = trimesh->z;
    shape->trimesh = trimesh;
    shape->canfree = canfree;

    return shape;
failure:
    Phy_FreeShape (shape);
    SCEE_LogSrc ();
    return NULL;
}
PhysicsShape* Phy_NewVoxelTerrainShape (SCE_SVoxelWorld *vw, float unit, int width)
{
    PhyTerrainShape *colshape = NULL;
    PhysicsShape *shape = (PhysicsShape*)SCE_malloc (sizeof *shape);
    if (!shape)
        goto failure;
    Phy_InitShape (shape);

    colshape = new PhyTerrainShape (vw, unit, width);
    shape->shape = colshape;

    return shape;
failure:
    Phy_FreeShape (shape);
    SCEE_LogSrc ();
    return NULL;
}
// TODO: not static nor in the header..?
PhysicsShape* Phy_NewTriMeshShapeCopy (PhysicsShape *ps, int canfree)
{
    btScaledBvhTriangleMeshShape *scb = NULL;
    btBvhTriangleMeshShape *thebvh = NULL;
    PhysicsShape *shape = (PhysicsShape*)SCE_malloc (sizeof *shape);
    if (!shape)
        goto fail;
    Phy_InitShape (shape);

    shape->canfree_parent = canfree;
    shape->parent = ps;
    shape->shape = NULL;        // lol
    shape->dynamic = ps->dynamic;
    shape->trimesh = ps->trimesh;
    shape->x = ps->trimesh->x;
    shape->y = ps->trimesh->y;
    shape->z = ps->trimesh->z;
    shape->canfree = SCE_FALSE;

    thebvh = (btBvhTriangleMeshShape*)ps->shape;
    scb = new btScaledBvhTriangleMeshShape (thebvh,
                                            btVector3 (1.0, 1.0, 1.0));
    shape->scaledbvh = (void*)scb;
    return shape;
fail:
    SCEE_LogSrc ();
    return NULL;
}
static PhysicsShape* Phy_LoadStaticTriMeshShape (const char*, int);

PhysicsShape* Phy_NewTriMeshShapeFromFile (const char *fname, int force)
{
    PhysicsShape *shape = NULL;
    PhysicsShape *ps = NULL;
    if (!(ps = Phy_LoadStaticTriMeshShape (fname, force))) goto fail;
    if (!(shape = Phy_NewTriMeshShapeCopy (ps, SCE_TRUE))) goto fail;
    return shape;
fail:
    SCEE_LogSrc ();
    return NULL;
}
void Phy_FreeShape (PhysicsShape *shape)
{
    if (shape) {
        if (SCE_Resource_Free (shape)) {
            Phy_ClearShape ((PhysicsShape*)shape);
            SCE_free (shape);
        }
    }
}


static PhysicsTriMesh* Phy_LoadTriMesh (const char*, int, int);

static void*
Phy_LoadStaticTriMeshShapeResource (const char *fname, int force, void *data)
{
    PhysicsShape *shape = NULL;
    PhysicsTriMesh *trimesh = NULL;
    (void)data;
    if (force > 0)
        force--;
    if (!(trimesh = Phy_LoadTriMesh (fname, SCE_FALSE, force))) goto fail;
    if (!(shape = Phy_NewTriMeshShape (SCE_FALSE, trimesh,
                                       SCE_FALSE, SCE_TRUE)))
        goto fail;
    return shape;
fail:
    SCEE_LogSrc ();
    return NULL;
}
// loads a static, concave and scalable trimesh shape
static PhysicsShape*
Phy_LoadStaticTriMeshShape (const char *fname, int force)
{
    return (PhysicsShape*)
        SCE_Resource_Load (resource_staticshape_type, fname, force, NULL);
}




static void Phy_ListFreeShape (void *shape)
{
    Phy_FreeShape ((PhysicsShape*)shape);
}
static void Phy_InitShapes (PhysicsShapes *shapes)
{
    shapes->shape = NULL;
    shapes->x = shapes->y = shapes->z = 0.0;
    SCE_List_Init (&shapes->shapes);
    SCE_List_SetFreeFunc (&shapes->shapes, Phy_ListFreeShape);
    shapes->built = SCE_FALSE;
}
PhysicsShapes* Phy_NewShapes (void)
{
    PhysicsShapes *shapes = NULL;
    if (!(shapes = (PhysicsShapes*)SCE_malloc (sizeof *shapes))) {
        SCEE_LogSrc ();
        return NULL;
    }
    Phy_InitShapes (shapes);
    return shapes;
}
PhysicsShapes* Phy_NewShapesFromShape (const SCE_TMatrix4 mat,
                                       PhysicsShape *shape)
{
    PhysicsShapes *shapes = NULL;
    if (!(shapes = Phy_NewShapes ()))
        goto fail;
    Phy_AddShape (shapes, mat, shape);
    return shapes;
fail:
    SCEE_LogSrc ();
    return NULL;
}
void Phy_FreeShapes (PhysicsShapes *shapes)
{
    if (shapes) {
        unsigned int n = SCE_List_GetLength (&shapes->shapes);
        if (n > 1) {
            btCompoundShape *com = (btCompoundShape*)shapes->shape;
            delete com;
        }
        SCE_List_Clear (&shapes->shapes);
        SCE_free (shapes);
    }
}

void Phy_DefaultUpdateFunc (Physics *phy, SCE_TMatrix4 m)
{
    SCE_Matrix4_Copy (phy->mat, m);
}

static void Phy_Init (Physics *phy)
{
    phy->type = PHY_STATIC_BODY;
    phy->typeobj = 0;
    phy->typemask = 0;
    phy->collide_with = ~0;     // collide with everything
    phy->mass = 1.0;
    phy->friction = 0.2;
    phy->body = NULL;
    phy->shapes = NULL;
    phy->canfree_shapes = SCE_TRUE;
    phy->motionstate = NULL;

    phy->updatefunc = Phy_DefaultUpdateFunc;

    SCE_Matrix4_Identity (phy->mat);
    SCE_Matrix4_Identity (phy->inv_mat);
    phy->updated = SCE_FALSE;

    phy->mutex = NULL;

    phy->is_static = SCE_TRUE;
    phy->userdata = NULL;
    phy->rho = 0.0;
    phy->use_rho = SCE_FALSE;
    phy->world = NULL;
    SCE_List_InitIt (&phy->it);
    SCE_List_SetData (&phy->it, phy);
}


Physics* Phy_New (PhyBodyType type)
{
//    btRigidBody::btRigidBodyConstructionInfo rbinfo (0, 0, 0);
    PhyMotionState *motionstate;
    Physics *phy = (Physics*)SCE_malloc (sizeof *phy);
    if (!phy)
        goto fail;
    motionstate = new PhyMotionState;
    if (!motionstate) {
        cppallocfail;
        goto fail;
    }

    Phy_Init (phy);

    motionstate->m_phy = phy;
    phy->type = type;
    phy->motionstate = motionstate;

    return phy;
fail:
    Phy_Free (phy);
    SCEE_LogSrc ();
    return NULL;
}

void Phy_Free (Physics *phy)
{
    if (phy) {
        if (SCE_Resource_Free (phy)) {
            SCE_List_Remove (&phy->it);
            if (phy->world)
                Phy_Remove (phy);
            if (phy->body) {
                btRigidBody *body = (btRigidBody*)phy->body;
                delete body;
            }
            if (phy->canfree_shapes)
                Phy_FreeShapes (phy->shapes);
            SCE_free (phy);
        }
    }
}


/**
 * \brief Adds a new shape to a list of shapes
 * \param mat can be NULL
 * \param shape the shape to add
 * \param canfree can we free \p shape or has already an owner?
 */
void Phy_AddShape (PhysicsShapes *shapes, const SCE_TMatrix4 mat,
                   PhysicsShape *shape)
{
    if (mat)
        SCE_Matrix4_Copy (shape->matrix, mat);
    SCE_List_Appendl (&shapes->shapes, &shape->it);
}
/**
 * \brief Builds a set of shapes
 * \param gscale global scale of the body
 */
int Phy_BuildShapes (PhysicsShapes *shapes, SCE_TVector3 gscale)
{
    unsigned int n = 0;
    SCE_SListIterator *it = NULL;

    if (shapes->built)
        return SCE_OK;

    n = SCE_List_GetLength (&shapes->shapes);
    if (n == 0) {
        SCEE_Log (SCE_INVALID_OPERATION);
        SCEE_LogMsg ("cannot build a shape group with no shape");
        return SCE_ERROR;
    } else if (n == 1) {
        // no compound
        PhysicsShape *shape = NULL;
        shape = (PhysicsShape*)SCE_List_GetData (SCE_List_GetFirst
                                                 (&shapes->shapes));

        // avoid scaling in the matrix
        SCE_TVector3 scale;

        SCE_Matrix4_GetScale (shape->matrix, scale);
        SCE_Matrix4_NoScaling (shape->matrix);

        SCE_Vector3_Operator2v (scale, =, scale, *, gscale);

        btCollisionShape *cs = NULL;
        if (shape->scaledbvh)
            cs = (btCollisionShape*)shape->scaledbvh;
        else
            cs = (btCollisionShape*)shape->shape;

        cs->setLocalScaling (btVector3 (scale[0], scale[1], scale[2]));
        shapes->shape = cs;

        shapes->x = shape->x * scale[0];
        shapes->y = shape->y * scale[1];
        shapes->z = shape->z * scale[2];

    } else {
        // compound is required
        btCompoundShape *com = new btCompoundShape (true);
        if (!com) {
            cppallocfail;
            goto fail;
        }
        SCE_List_ForEach (it, &shapes->shapes) {
            PhysicsShape *shape = (PhysicsShape*)SCE_List_GetData (it);

            // avoid scaling in the matrix
            SCE_TVector3 scale, vec;

            SCE_Matrix4_GetScale (shape->matrix, scale);
            SCE_Matrix4_NoScaling (shape->matrix);

            // TODO: global scaling shouldn't be taken into account,
            // only apply it to final shape.
            SCE_Vector3_Operator2v (scale, =, scale, *, gscale);
            btCollisionShape *cs = (btCollisionShape*)shape->shape;
            cs->setLocalScaling (btVector3 (scale[0], scale[1], scale[2]));

            btTransform trans;
            SCE_Matrix4_GetTranslation (shape->matrix, vec);
            SCE_Vector3_Operator1v (vec, *=, gscale);
            SCE_Matrix4_SetTranslation (shape->matrix, vec);
            Phy_TransformFromMatrix (trans, shape->matrix);
            com->addChildShape (trans, cs);
            // TODO: bad =>
            shapes->x += shape->x;
            shapes->y += shape->y;
            shapes->z += shape->z;
        }
        shapes->shape = com;
    }

    shapes->built = SCE_TRUE;
    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}


PhyBodyType Phy_GetType (Physics *phy)
{
    return phy->type;
}

static void Phy_SetObjMaskToBody (Physics *phy)
{
    if (phy->world) {
        PhyWorld *world = phy->world;
        btRigidBody *body = (btRigidBody*)phy->body;
        Phy_Remove (phy);
        Phy_Add (world, phy);
    }
}
void Phy_SetObjType (Physics *phy, int type)
{
    phy->typeobj = type;
    phy->typemask = 1 << type;  // hope 'type' is within [1, 15]
    Phy_SetObjMaskToBody (phy);
}
int Phy_GetObjType (Physics *phy)
{
    return phy->typeobj;
}
// internal use
void Phy_SetTypeMask (Physics *phy, short type)
{
    phy->typemask = type;
}
short Phy_GetTypeMask (Physics *phy)
{
    return phy->typemask;
}

void Phy_CollideWith (Physics *phy, short mask)
{
    phy->collide_with = mask;
}
void Phy_AddCollide (Physics *phy, int type)
{
    phy->collide_with |= 1 << type;
}
void Phy_RemoveCollide (Physics *phy, int type)
{
    phy->collide_with &= ~(1 << type);
}
void Phy_CollideWithEverything (Physics *phy)
{
    phy->collide_with = ~0;
}


void Phy_SetData (Physics *phy, void *data)
{
    phy->userdata = data;
}
void* Phy_GetData (Physics *phy)
{
    return phy->userdata;
}

void Phy_SetMass (Physics *phy, float mass)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    phy->is_static = (mass == 0.0 ? SCE_TRUE : SCE_FALSE);
    phy->mass = mass;
    if (body != NULL) {
        PhyWorld *world = phy->world;
        if (world)
            Phy_Remove (phy);
        btVector3 localinertia (0.0, 0.0, 0.0);
        if (!phy->is_static) {
            btCollisionShape *shape = (btCollisionShape*)phy->shapes->shape;
            shape->calculateLocalInertia (mass, localinertia);
        }
        body->setMassProps (btScalar (mass), localinertia);
        if (world)
            Phy_Add (world, phy);
    }
    // deprecated... ?
    phy->is_static = (mass == 0.0 ? SCE_TRUE : SCE_FALSE);
}
float Phy_GetMass (Physics *phy)
{
    return phy->mass;
}

void Phy_SetFriction (Physics *phy, float friction)
{
    phy->friction = friction; // TODO: doesn't affect friction if body is built
}
float Phy_GetFriction (Physics *phy)
{
    return phy->friction;
}


void Phy_SetUpdateFunction (Physics *phy, PhyUpdateFunc f)
{
    phy->updatefunc = (f ? f : Phy_DefaultUpdateFunc);
}

void Phy_GetMatrix4v (Physics *phy, SCE_TMatrix4 m)
{
    if (!phy->mutex) {
        SCE_Matrix4_Copy (m, phy->mat);
    } else {
        if (!pthread_mutex_lock (phy->mutex)) {
            SCE_Matrix4_Copy (m, phy->mat);
            pthread_mutex_unlock (phy->mutex);
        }
    }
}
void Phy_GetInvMatrix4v (Physics *phy, SCE_TMatrix4 m)
{
    if (!phy->updated) {
        SCE_TMatrix4 mat;
        Phy_GetMatrix4v (phy, mat);
        SCE_Matrix4_Inverse (mat, phy->inv_mat);
        phy->updated = SCE_TRUE;
    }
    SCE_Matrix4_Copy (m, phy->inv_mat);
}
void Phy_GetPos (Physics *phy, Position *pos)
{
    SCE_TMatrix4 mat;
    Phy_GetMatrix4v (phy, mat);
    Pos_Init (pos);
    Pos_SetFromMatrix4x3 (pos, mat);
    Phy_GetLinearVelocityRelv (phy, pos->lvel);
    Phy_GetAngularVelocityRelv (phy, pos->avel);
}

void Phy_SetMutex (Physics *phy, pthread_mutex_t *mutex)
{
    phy->mutex = mutex;
}

void Phy_SetShapes (Physics *phy, PhysicsShapes *shapes)
{
    Phy_FreeShapes (phy->shapes);
    phy->shapes = shapes;
}
PhysicsShapes* Phy_GetShapes (Physics *phy)
{
    return phy->shapes;
}
#ifdef PHY_GET_MESH
SCE_SMesh* Phy_GetMesh (Physics *phy)
{
    PhyTerrainShape *shape = (PhyTerrainShape*)phy->shapes->shape;
    return shape->getMesh ();
}
#endif

void Phy_Activate (Physics *phy, int activated)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->setActivationState ((activated ? ACTIVE_TAG : WANTS_DEACTIVATION));
}
int Phy_IsActivated (Physics *phy)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    int s = body->getActivationState ();
    switch (s) {
    case ACTIVE_TAG:
    case DISABLE_DEACTIVATION: return SCE_TRUE;
    }
    return SCE_FALSE;
}
// TODO: check that
void Phy_SetPersistentActivation (Physics *phy, int persistent)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    if (persistent)
        body->forceActivationState (DISABLE_DEACTIVATION);
    else
        body->forceActivationState (ACTIVE_TAG);
}

#if 0
PhysicsShape* Phy_GetMainShape (Physics *phy)
{
}
#endif

int Phy_IsStatic (Physics *phy)
{
    return phy->is_static;
}


void Phy_SetPosition (Physics *phy, float x, float y, float z)
{
    btTransform t;
//    btRigidBody *body = (btRigidBody*)phy->body;
    btMotionState *mstate = (btMotionState*)phy->motionstate;
    mstate->getWorldTransform (t);
    t.setOrigin (btVector3 (x, y, z));
    mstate->setWorldTransform (t);
}
void Phy_SetPositionv (Physics *phy, SCE_TVector3 v)
{
    Phy_SetPosition (phy, v[0], v[1], v[2]);
}
#if 0
void Phy_SetRotation (Physics *phy, float x, float y, float z)
void Phy_SetRotationv (Physics *phy, SCE_TVector3 v)
#endif
void Phy_SetOrientation (Physics *phy, SCE_TQuaternion q)
{
    btTransform t;
    btMotionState *mstate = (btMotionState*)phy->motionstate;
    mstate->getWorldTransform (t);
    t.setRotation (btQuaternion (q[0], q[1], q[2], q[3]));
    mstate->setWorldTransform (t);
}
void Phy_SetMatrix (Physics *phy, SCE_TMatrix4 m)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    btTransform t;
    btMotionState *mstate = (btMotionState*)phy->motionstate;
    PhyWorld *w = phy->world;
    if (w)
        Phy_Remove (phy);
//    mstate->getWorldTransform (t); // NOTE: lol?
    Phy_TransformFromMatrix (t, m);
    mstate->setWorldTransform (t);
    if (body)
        body->setMotionState (mstate);
    if (w)
        Phy_Add (w, phy);
}

static int Phy_InstanciateShapes (Physics *p1, Physics *p2)
{
    PhysicsShape *shape = NULL;
    shape = (PhysicsShape*)SCE_List_GetData(SCE_List_GetFirst(&p1->shapes->shapes));
    if (!shape->trimesh || shape->dynamic || p2->type == PHY_DYNAMIC_BODY) {
        // TODO: just do a copy of the shapes (instanciation of dynamic
        // trimeshes or simple shapes like boxes, spheres & co
        p2->shapes = p1->shapes;
        p2->canfree_shapes = SCE_FALSE;
    } else {
        if (shape->scaledbvh)
            shape = shape->parent;
        PhysicsShape *scaledbvh = Phy_NewTriMeshShapeCopy (shape, SCE_FALSE);
        p2->shapes = Phy_NewShapesFromShape (sce_matrix4_id, scaledbvh);
        if (!p2->shapes) {
            SCEE_LogSrc ();
            return SCE_ERROR;
        }
    }
    return SCE_OK;

    // note that with this method, if p1 is cleared p2 remains with an
    // invalid "shapes", lol.
}

// p2 will use the same shapes but with another body
int Phy_Instanciate (Physics *p1, Physics *p2)
{
    if (Phy_InstanciateShapes (p1, p2) < 0) {
        SCEE_LogSrc ();
        return SCE_ERROR;
    }
    p2->mass = p1->mass;
    p2->friction = p1->friction;
    return SCE_OK;
}
Physics* Phy_NewInstanciate (Physics *p1, PhyBodyType type)
{
    Physics *p2 = NULL;
    if (!(p2 = Phy_New (type))) {
        SCEE_LogSrc ();
        return NULL;
    }
    Phy_Instanciate (p1, p2);
    return p2;
}


int Phy_Build (Physics *phy)
{
    btCollisionShape *shape;
    btRigidBody *body;
    btRigidBody::btRigidBodyConstructionInfo rbinfo (0, 0, 0);
    btMotionState *mstate = NULL;
    btTransform trans;
    SCE_TMatrix4 mat;
    SCE_TVector3 scale;

    // avoid scaling in the matrix
    mstate = (btMotionState*)phy->motionstate;
    mstate->getWorldTransform (trans);
    Phy_MatrixFromTransform (mat, trans);
    SCE_Matrix4_GetScale (mat, scale);

    if (Phy_BuildShapes (phy->shapes, scale) < 0)
        goto fail;

    shape = (btCollisionShape*)phy->shapes->shape;
    shape->setLocalScaling (btVector3 (scale[0], scale[1], scale[2]));

    SCE_Matrix4_NoScaling (mat);
    Phy_TransformFromMatrix (trans, mat);
    mstate->setWorldTransform (trans);

    if (phy->type != PHY_DYNAMIC_BODY)
        phy->mass = 0.0;
    rbinfo.m_mass = phy->mass;
    shape->calculateLocalInertia (phy->mass, rbinfo.m_localInertia);
    rbinfo.m_motionState = mstate;
    rbinfo.m_collisionShape = shape;
    rbinfo.m_friction = phy->friction;    // wesh
    rbinfo.m_restitution = 0.0;
    body = new btRigidBody (rbinfo);
    if (!body) {
        cppallocfail;
        goto fail;
    }
    if (phy->type == PHY_KINEMATIC_BODY) {
        body->setCollisionFlags (body->getCollisionFlags() |
                                 btCollisionObject::CF_KINEMATIC_OBJECT);
        body->setActivationState (DISABLE_DEACTIVATION);
    }

    body->setUserPointer (phy);
    phy->body = body;
    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}

void Phy_Add (PhyWorld *world, Physics *phy)
{
    btDiscreteDynamicsWorld *w = (btDiscreteDynamicsWorld*)world->world;
    btRigidBody *body = (btRigidBody*)phy->body;
    if (phy->typemask != 0)
        w->addRigidBody (body, phy->typemask, phy->collide_with);
    else
        w->addRigidBody (body);
    phy->world = world;
    if (phy->use_rho)
        SCE_List_Appendl (&world->fluid_bodies, &phy->it);
}
void Phy_Remove (Physics *phy)
{
    btDiscreteDynamicsWorld *w = (btDiscreteDynamicsWorld*)phy->world->world;
    btRigidBody *body = (btRigidBody*)phy->body;
    w->removeRigidBody (body);
    phy->world = NULL;
}



void Phy_SetLinearVelAbs (Physics *phy, float x, float y, float z)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->setLinearVelocity (btVector3 (x, y, z));
}
void Phy_SetLinearVelAbsv (Physics *phy, const SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->setLinearVelocity (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularVelAbs (Physics *phy, float x, float y, float z)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->setAngularVelocity (btVector3 (x, y, z));
}
void Phy_SetAngularVelAbsv (Physics *phy, const SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->setAngularVelocity (btVector3 (v[0], v[1], v[2]));
}


// project in global space functions
static void Phy_ProjectInGlobalSpaceCopyv (Physics *phy, SCE_TVector3 in)
{
    SCE_TMatrix4 mat;
    Phy_GetMatrix4v (phy, mat);
    SCE_Matrix4_MulV3Copyw (mat, in, 0.0f); // kick translation
}
static void Phy_ProjectInGlobalSpacev (Physics *phy, const SCE_TVector3 in,
                                       SCE_TVector3 out)
{
    SCE_TMatrix4 mat;
    Phy_GetMatrix4v (phy, mat);
    SCE_Matrix4_MulV3w (mat, (float*)in, 0.0f, out); // kick translation
}
static void Phy_ProjectInGlobalSpace (Physics *phy, float x, float y, float z,
                                      SCE_TVector3 v)
{
    SCE_Vector3_Set (v, x, y, z);
    Phy_ProjectInGlobalSpaceCopyv (phy, v);
}
// project in local space functions
static void Phy_ProjectInLocalSpaceCopyv (Physics *phy, SCE_TVector3 in)
{
    SCE_TMatrix4 mat;
    Phy_GetInvMatrix4v (phy, mat);
    SCE_Matrix4_MulV3Copyw (mat, in, 0.0f); // kick translation
}
static void Phy_ProjectInLocalSpacev (Physics *phy, const SCE_TVector3 in,
                                      SCE_TVector3 out)
{
    SCE_TMatrix4 mat;
    Phy_GetInvMatrix4v (phy, mat);
    SCE_Matrix4_MulV3w (mat, (float*)in, 0.0f, out); // kick translation
}
static void Phy_ProjectInLocalSpace (Physics *phy, float x, float y, float z,
                                     SCE_TVector3 v)
{
    SCE_Vector3_Set (v, x, y, z);
    Phy_ProjectInLocalSpaceCopyv (phy, v);
}

void Phy_SetLinearVelRel (Physics *phy, float x, float y, float z)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpace (phy, x, y, z, v);
    body->setLinearVelocity (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetLinearVelRelv (Physics *phy, const SCE_TVector3 in)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpacev (phy, in, v);
    body->setLinearVelocity (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularVelRel (Physics *phy, float x, float y, float z)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpace (phy, x, y, z, v);
    body->setAngularVelocity (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularVelRelv (Physics *phy, const SCE_TVector3 in)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpacev (phy, in, v);
    body->setAngularVelocity (btVector3 (v[0], v[1], v[2]));
}


void Phy_SetLinearImpulseAbs (Physics *phy, float x, float y, float z)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->applyCentralImpulse (btVector3 (x, y, z));
}
void Phy_SetLinearImpulseAbsv (Physics *phy, const SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->applyCentralImpulse (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularImpulseAbs (Physics *phy, float x, float y, float z)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->applyTorqueImpulse (btVector3 (x, y, z));
}
void Phy_SetAngularImpulseAbsv (Physics *phy, const SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    body->applyTorqueImpulse (btVector3 (v[0], v[1], v[2]));
}

void Phy_SetLinearImpulseRel (Physics *phy, float x, float y, float z)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpace (phy, x, y, z, v);
    body->applyCentralImpulse (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetLinearImpulseRelv (Physics *phy, const SCE_TVector3 in)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpacev (phy, in, v);
    body->applyCentralImpulse (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularImpulseRel (Physics *phy, float x, float y, float z)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpace (phy, x, y, z, v);
    body->applyTorqueImpulse (btVector3 (v[0], v[1], v[2]));
}
void Phy_SetAngularImpulseRelv (Physics *phy, const SCE_TVector3 in)
{
    SCE_TVector3 v;
    btRigidBody *body = (btRigidBody*)phy->body;
    Phy_ProjectInGlobalSpacev (phy, in, v);
    body->applyTorqueImpulse (btVector3 (v[0], v[1], v[2]));
}


void Phy_GetLinearVelocityAbsv (Physics *phy, SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    btVector3 btv = body->getLinearVelocity ();
    Phy_CopyVector3 (v, btv);
}
void Phy_GetAngularVelocityAbsv (Physics *phy, SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    btVector3 btv = body->getAngularVelocity (); 
    Phy_CopyVector3 (v, btv);
}

void Phy_GetLinearVelocityRelv (Physics *phy, SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    btVector3 btv = body->getLinearVelocity ();
    Phy_CopyVector3 (v, btv);
    Phy_ProjectInLocalSpaceCopyv (phy, v);
}
// TODO: is this correct? what does the angular velocity vector mean exactly?
void Phy_GetAngularVelocityRelv (Physics *phy, SCE_TVector3 v)
{
    btRigidBody *body = (btRigidBody*)phy->body;
    btVector3 btv = body->getAngularVelocity ();
    Phy_CopyVector3 (v, btv);
    Phy_ProjectInLocalSpaceCopyv (phy, v);
}


void Phy_SetRho (Physics *phy, float rho)
{
    if (rho != 0.0) {
        if (!phy->use_rho) {
            if (phy->world)
                SCE_List_Appendl (&phy->world->fluid_bodies, &phy->it);
            phy->use_rho = SCE_TRUE;
        }
    } else {
        if (phy->use_rho) {
            SCE_List_Remove (&phy->it);
            phy->use_rho = SCE_FALSE;
        }
    }
    phy->rho = rho;
}


static void Phy_UpdateBodiesInFluid (SCE_SList *list, float secs)
{
    btRigidBody *body;
    Physics *phy = NULL;
    SCE_TVector3 vel;
    SCE_TVector3 friction;
    SCE_SListIterator *it = NULL;
    // update bodies that have friction with a fluid
    SCE_List_ForEach (it, list) {
        phy = (Physics*)SCE_List_GetData (it);
        body = (btRigidBody*)phy->body;
        SCE_Vector3_Set (friction,phy->shapes->x,phy->shapes->y,phy->shapes->z);
        // F = - 1/2 * rho * S * v * ~secs
        Phy_GetLinearVelocityRelv (phy, vel);
        SCE_Vector3_Operator2 (vel, *=, friction, *, secs * 0.5 * -phy->rho);
        Phy_SetLinearImpulseRelv (phy, vel);
        // angular friction
        Phy_GetAngularVelocityRelv (phy, vel);
        SCE_Vector3_Operator2 (vel, *=, friction, *, secs * 0.4 * -phy->rho);
        vel[0] *= (friction[1] * 0.5 + friction[2] * 0.5);
        vel[1] *= (friction[2] * 0.5 + friction[0] * 0.5);
        vel[2] *= (friction[1] * 0.5 + friction[0] * 0.5);
        Phy_SetAngularImpulseRelv (phy, vel);
    }
}

void Phy_UpdateWorld (PhyWorld *world, float secs)
{
    btDiscreteDynamicsWorld *w = (btDiscreteDynamicsWorld*)world->world;
    Phy_UpdateBodiesInFluid (&world->fluid_bodies, secs);
#define PHY_TIMESTEP (1.0/60.0)
    w->stepSimulation (secs, PHY_MAX_SUBSTEPS, PHY_TIMESTEP);
#ifdef TL_PHY_DEBUG
    w->debugDrawWorld ();
#endif
}


static void Phy_NearCallback (btBroadphasePair& pair,
                              btCollisionDispatcher& dis,
                              const btDispatcherInfo& info)
{
    PhyCollision col;
    btCollisionObject *c1, *c2;
    Physics *p1 = NULL, *p2 = NULL;
    c1 = (btCollisionObject*)pair.m_pProxy0->m_clientObject;
    c2 = (btCollisionObject*)pair.m_pProxy1->m_clientObject;
    p1 = (Physics*)c1->getUserPointer ();
    p2 = (Physics*)c2->getUserPointer ();
    PhyWorld *world = p1->world;
    col.pair = (void*)&pair;
    col.dispatcher = (void*)&dis;
    col.dispatcherinfo = (void*)&info;
    col.done = SCE_FALSE;
    col.need = PHY_MAYBE;
    col.inverse = SCE_FALSE;
    world->near_callback (world, p1, p2, &col, world->near_data);
}

void Phy_SetCollisionCallback (PhyWorld *world, PhyCollisionCallbackFunc f,
                               void *data)
{
    btCollisionDispatcher *disp = (btCollisionDispatcher*)world->disp;
    world->near_callback = f;
    world->near_data = data;
    disp->setNearCallback ((f ? Phy_NearCallback : NULL));
}

int Phy_NeedsCollision (PhyCollision *col)
{
    if (col->need == PHY_MAYBE) {
        btBroadphasePair *pair = (btBroadphasePair*)col->pair;
        btCollisionDispatcher *disp = (btCollisionDispatcher*)col->dispatcher;
        btCollisionObject* obj0 =
            (btCollisionObject*)pair->m_pProxy0->m_clientObject;
        btCollisionObject* obj1 =
            (btCollisionObject*)pair->m_pProxy1->m_clientObject;
        col->need = disp->needsCollision (obj0, obj1) ? PHY_YES : PHY_NO;
    }
    return col->need;
}

void Phy_ProcessCollision (PhyCollision *col)
{
    if (col->done)
        return;

    btBroadphasePair *pair = (btBroadphasePair*)col->pair;
    btCollisionDispatcher *disp = (btCollisionDispatcher*)col->dispatcher;
    btDispatcherInfo *dispinfo = (btDispatcherInfo*)col->dispatcherinfo;
    btCollisionObject* object0 =
        (btCollisionObject*)pair->m_pProxy0->m_clientObject;
    btCollisionObject* object1 =
        (btCollisionObject*)pair->m_pProxy1->m_clientObject;
    btCollisionObjectWrapper obj0(0, object0->getCollisionShape (), object0,
                                  object0->getWorldTransform ());
    btCollisionObjectWrapper obj1(0, object1->getCollisionShape (), object1,
                                  object1->getWorldTransform ());

    //dispatcher will keep algorithms persistent in the collision pair
    if (!pair->m_algorithm) {
        pair->m_algorithm = disp->findAlgorithm (&obj0, &obj1);
        if (!pair->m_algorithm)
            return;
    }

    btManifoldResult cpr (&obj0, &obj1); // contact point result
    if (dispinfo->m_dispatchFunc == btDispatcherInfo::DISPATCH_DISCRETE) {
        // discrete collision detection query
        pair->m_algorithm->processCollision (&obj0, &obj1, *dispinfo, &cpr);
    } else {
        //continuous collision detection query, time of impact (toi)
        btScalar toi = pair->m_algorithm->calculateTimeOfImpact
            (object0, object1, *dispinfo, &cpr);
        if (dispinfo->m_timeOfImpact > toi)
            dispinfo->m_timeOfImpact = toi;
    }

    col->done = SCE_TRUE;
}

static void Phy_InverseCollisionInfo (PhyCollisionInfo *info)
{
    PhyCollisionPoint *p = info->p1;
    info->p1 = info->p2;
    info->p2 = p;
    SCE_Vector3_Operator1 (info->normal, *=, -1.0);
}
static void Phy_InverseCollisionInfov (unsigned int n, PhyCollisionInfo *info)
{
    unsigned int i;
    for (i = 0; i < n; i++)
        Phy_InverseCollisionInfo (&info[i]);
}
int Phy_GetCollisionInfov (PhyCollision *col, unsigned int num,
                           PhyCollisionInfo *info, int options)
{
    unsigned int i, j, n;
    btManifoldArray marray;
    btBroadphasePair *pair = (btBroadphasePair*)col->pair;

    if (pair->m_algorithm)
        pair->m_algorithm->getAllContactManifolds (marray);

    n = 0;
    for (j = 0; j < marray.size () && n < num; j++) {
        btPersistentManifold *pmanifold = marray[j];
        unsigned int nc = (unsigned int)pmanifold->getNumContacts ();
        // get points
        for (i = 0; i < nc && n < num; i++, n++) {
            const btManifoldPoint p = pmanifold->getContactPoint (i);
            // copy data, according to options
            if (PHY_COPY_POINT1 & options) {
                Phy_CopyVector3 (info[n].p1->abs_pos, p.m_positionWorldOnA);
                Phy_CopyVector3 (info[n].p1->rel_pos, p.m_localPointA);
            }
            if (PHY_COPY_POINT2 & options) {
                Phy_CopyVector3 (info[n].p2->abs_pos, p.m_positionWorldOnB);
                Phy_CopyVector3 (info[n].p2->rel_pos, p.m_localPointB);
            }
            Phy_CopyVector3 (info[n].normal, p.m_normalWorldOnB);
            // info[n].distance = p.m_distance1;
            info[n].distance = p.getDistance ();
            n++;
        }
    }
    if (col->inverse)
        Phy_InverseCollisionInfov (n, info);
    return n;
}

static void* Phy_LoadTriMeshResource (const char *fname, int force, void *data)
{
    SCE_SGeometry *geom = NULL;
    PhysicsTriMesh *trimesh = NULL;
    if (force > 0)
        force--;                // downgrading depth
    if (!(geom = SCE_Geometry_Load (fname, force)))
        goto fail;
    if (!(trimesh = Phy_NewTriMesh (geom, *(int*)data, SCE_TRUE)))
        goto fail;
    return trimesh;
fail:
    SCE_Geometry_Delete (geom);
    SCEE_LogSrc ();
    return NULL;
}
static PhysicsTriMesh* Phy_LoadTriMesh (const char *fname, int convex,
                                        int force)
{
    PhysicsTriMesh *trimesh = NULL;
    trimesh = (PhysicsTriMesh*)SCE_Resource_Load (resource_trimesh_type,
                                                  fname, force, &convex);
    if (!trimesh)
        SCEE_LogSrc ();
    return trimesh;
}
