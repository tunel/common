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

#include <SCE/utils/SCEUtils.h>
#include <SCE/core/SCECore.h>
#include "terrainbrush.h"

void TBrush_Init (TerrainBrush *tb)
{
    tb->size = 1;
    tb->mode = TBRUSH_REMOVE;
    tb->fun = NULL;
    tb->udata = NULL;
}
void TBrush_Clear (TerrainBrush *tb)
{
    /* maybe call some user free callback?.. nah. */
}
TerrainBrush* TBrush_New (void)
{
    TerrainBrush *tb = NULL;
    if (!(tb = SCE_malloc (sizeof *tb)))
        SCEE_LogSrc ();
    else
        TBrush_Init (tb);
    return tb;
}
void TBrush_Free (TerrainBrush *tb)
{
    if (tb) {
        TBrush_Clear (tb);
        SCE_free (tb);
    }
}

void TBrush_SetSize (TerrainBrush *tb, unsigned int size)
{
    tb->size = size;
}
unsigned int TBrush_GetSize (TerrainBrush *tb)
{
    return tb->size;
}

void TBrush_SetMode (TerrainBrush *tb, TerrainBrushMode mode)
{
    tb->mode = mode;
}
TerrainBrushMode TBrush_GetMode (TerrainBrush *tb)
{
    return tb->mode;
}

void TBrush_SetData (TerrainBrush *tb, void *data)
{
    tb->udata = data;
}
void* TBrush_GetData (TerrainBrush *tb)
{
    return tb->udata;
}
void TBrush_SetFunc (TerrainBrush *tb, TerrainBrushFunc fun)
{
    tb->fun = fun;
}

void TBrush_Apply (TerrainBrush *tb, const SCE_TVector3 pos, SCEuint w,
                   SCEuint h, SCEuint d, SCEubyte *data)
{
    if (tb->fun)
        tb->fun (tb, pos, w, h, d, data);
}


void TBrush_SphereFunc (TerrainBrush *tb, const SCE_TVector3 center, SCEuint w,
                        SCEuint h, SCEuint d, SCEubyte *buf)
{
    SCE_TVector3 pos;
    float r, dist;
    SCEulong x, y, z;
    int fill;
#if 0
    /* hope that tb->size doesnt exceed 14 :> */
    SCEubyte buf[32*32*32] = {0};

    memcpy (buf, data, w * h * d);
#endif

    r = TBrush_GetSize (tb);

    switch (TBrush_GetMode (tb)) {
    case TBRUSH_ADD: fill = SCE_TRUE; break;
    case TBRUSH_REMOVE: fill = SCE_FALSE; break;
    default: fill = SCE_FALSE;
    }

    for (z = 0; z < d; z++) {
        for (y = 0; y < h; y++) {
            for (x = 0; x < w; x++) {
                unsigned char p;
                size_t offset = w * (h * z + y) + x;

                offset *= SCE_VOCTREE_VOXEL_ELEMENTS;

                p = buf[offset];

                SCE_Vector3_Set (pos, x, y, z);
                dist = SCE_Vector3_Distance (pos, center);
                if (dist <= r + 1) {
                    float w = 0.0;
                    if (dist >= r - 1)
                        w = (dist - r + 1.0) / 2.0;

                    if (fill)
                        p = MAX (p, (1.0 - w) * 255);
                    else
                        p = MIN (p, w * 255);

                    buf[offset] = p;
                }
            }
        }
    }
}

void *TBrush_SphereData = NULL;
