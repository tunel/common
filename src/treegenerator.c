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

#include <SCE/core/SCECore.h>
#include "treegenerator.h"

static int vworld_seed (SCE_SVoxelWorld *vw, SCEuint level, long min_altitude,
                        SCE_TVector3 origin)
{
#define bdepth 10
    SCE_SLongRect3 r;
    SCEubyte buf[bdepth];
    long x, y, z;
    int i;
    int first = SCE_TRUE, loop = SCE_TRUE;

    x = origin[0];
    y = origin[1];
    z = origin[2];

    while (loop) {
        z -= bdepth;
        SCE_Rectangle3_SetFromOriginl (&r, x, y, z, 1, 1, bdepth);
        memset (buf, 0, bdepth);
        SCE_VWorld_GetRegion (vw, level, &r, buf);
        /* TODO: hardcoded usage of voxel data */
        if (first && buf[bdepth - 1] > 127)
            return SCE_FALSE;
        first = SCE_FALSE;
        for (i = bdepth - 1; i >= 0; i--) {
            if (buf[i] > 127) {
                z += i;
                loop = SCE_FALSE;
                break;
            }
        }
        if (z <= min_altitude)
            return SCE_FALSE;
    }

    origin[2] = z;

    return SCE_TRUE;
}

static float myrand (float min, float max)
{
    float r = rand ();
    r /= RAND_MAX;
    r *= (max - min);
    r += min;
    return r;
}

static void generatecloud (SCE_SVoxelGrid *grid, SCE_TVector3 *points,
                           size_t n_points, float coef)
{
    size_t i;
    float w, h, d;

    w = SCE_VGrid_GetWidth (grid);
    h = SCE_VGrid_GetHeight (grid);
    d = SCE_VGrid_GetDepth (grid);

    i = 0;
    while (i < n_points) {
        SCE_TVector3 p;
        float depth;

        if (myrand (0.0, 100.0) > 60.0)
            depth = myrand (d * 0.7, d - 1.0);
        else
            depth = myrand (d * 0.2, d * 0.7);
        SCE_Vector3_Set (p, myrand (0.0, w - 1.0), myrand (0.0, h - 1.0),
                         depth);
        /* TODO: hardcoded usage of voxel data */
        if (*SCE_VGrid_Offset (grid, p[0], p[1], p[2]) <= 40) {
            p[0] -= w / 2.0;
            p[1] -= w / 2.0;
            SCE_Vector3_Operator1 (p, *=, coef);
            SCE_Vector3_Copy (points[i], p);
            i++;
        }
    }
    /* set a few points for the trunk (which btw can cross terrain but whatever) */
    /* TODO: n_points is _probably_ higher than 5 */
    for (i = 1; i < 5; i++)
        SCE_Vector3_Set (points[i - 1], 0.0, 0.0, i * ((d * 0.2) / 5.0));
}

static int generatetree (SCE_SVoxelWorld *vw, SCEuint level,
                         const SCE_SLongRect3 *region, size_t n_points,
                         SCE_SForestTreeParameters *param, SCE_SForestTree *ft,
                         float vunit)
{
    SCE_TVector3 *points = NULL;
    size_t n_voxels;
    SCE_SVoxelGrid grid;
    SCEubyte *voxels = NULL;
    SCE_TVector3 origin;
    SCEulong w, h, d;
    float coef;

    n_voxels = SCE_Rectangle3_GetAreal (region);

    if (!(voxels = SCE_malloc (n_voxels)))
        goto fail;
    if (!(points = SCE_malloc (n_points * sizeof *points)))
        goto fail;

    memset (voxels, 0, n_voxels);
    SCE_VWorld_GetRegion (vw, level, region, voxels);

    SCE_VGrid_Init (&grid);
    w = SCE_Rectangle3_GetWidthl (region);
    h = SCE_Rectangle3_GetHeightl (region);
    d = SCE_Rectangle3_GetDepthl (region);
    SCE_VGrid_SetDimensions (&grid, w, h, d);
    SCE_VGrid_SetRaw (&grid, voxels);

    coef = vunit;
    while (level--)
        coef *= 2.0;
    generatecloud (&grid, points, n_points, coef);
    SCE_free (voxels);

    SCE_Vector3_Set (origin, 0.0, 0.0, 0.0);
    if (SCE_FTree_SpaceColonization (ft, param, origin, points, n_points) < 0)
        goto fail;

    SCE_free (points);

    return SCE_OK;
fail:
    SCE_free (voxels);
    SCE_free (points);
    SCEE_LogSrc ();
    return SCE_ERROR;
}


static int createtree (SCE_SVoxelWorld *vw, SCEuint level, long min_altitude,
                       SCE_TVector3 origin, const SCE_SLongRect3 *dim,
                       size_t n_points, SCE_SForestTreeParameters *param,
                       SCE_SForestTree *ft, float vunit)
{
    SCE_SLongRect3 region;

    if (!vworld_seed (vw, level, min_altitude, origin))
        return SCE_OK;
    region = *dim;
    SCE_Rectangle3_Movel (&region, origin[0], origin[1], origin[2]);

    if (generatetree (vw, level, &region, n_points, param, ft, vunit) < 0)
        goto fail;

    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}

int TGen_Generate (SCE_SVoxelWorld *vw, SCEuint level, SCE_TVector3 origin,
                   float size, float height, float vunit, SCE_SForestTree *ft)
{
    SCE_SLongRect3 r;
    SCE_SForestTreeParameters param;
    size_t n_points = 1000;
    SCEuint l;

    SCE_Rectangle3_Setl (&r, -size / 2.0, -size / 2.0, 0.0,
                              size / 2.0,  size / 2.0, height);

#define cub(a) ((a)*(a))
    param.grow_dist = 1.3;
    param.kill_dist = cub(1.0);
    param.radius = cub(10.0);
    param.max_nodes = 800;

    l = level;
    SCE_Vector3_Operator1 (origin, /=, vunit);
    while (l) {
        SCE_Vector3_Operator1 (origin, *=, 0.5);
        l--;
    }

    if (createtree (vw, level, 1, origin, &r, n_points, &param, ft, vunit) < 0)
        goto fail;

    l = level;
    SCE_Vector3_Operator1 (origin, *=, vunit);
    while (l) {
        SCE_Vector3_Operator1 (origin, *=, 2.0);
        l--;
    }

    SCE_FTree_CountNodes (ft);
    if (SCE_FTree_GetNumNodes (ft))
        SCE_FTree_ComputeRadius (ft, 1.);

    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}
