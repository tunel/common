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

#ifndef H_TERRAINBRUSH
#define H_TERRAINBRUSH

#include <SCE/utils/SCEUtils.h>

typedef enum {
    TBRUSH_ADD,
    TBRUSH_REMOVE
} TerrainBrushMode;

typedef struct terrainbrush TerrainBrush;

typedef void (*TerrainBrushFunc)(TerrainBrush*, const SCE_TVector3, SCEuint,
                                 SCEuint, SCEuint, SCEubyte*);

/* this structure is kinda ridiculous */
struct terrainbrush {
    unsigned int size;
    TerrainBrushMode mode;
    TerrainBrushFunc fun;
    void *udata;
};

void TBrush_Init (TerrainBrush*);
void TBrush_Clear (TerrainBrush*);
TerrainBrush* TBrush_New (void);
void TBrush_Free (TerrainBrush*);

void TBrush_SetSize (TerrainBrush*, unsigned int);
unsigned int TBrush_GetSize (TerrainBrush*);

void TBrush_SetMode (TerrainBrush*, TerrainBrushMode);
TerrainBrushMode TBrush_GetMode (TerrainBrush*);

void TBrush_SetData (TerrainBrush*, void*);
void* TBrush_GetData (TerrainBrush*);
void TBrush_SetFunc (TerrainBrush*, TerrainBrushFunc);

void TBrush_Apply (TerrainBrush*, const SCE_TVector3, SCEuint, SCEuint,
                   SCEuint, SCEubyte*);

/* some brushes :) */
void TBrush_SphereFunc (TerrainBrush*, const SCE_TVector3, SCEuint, SCEuint,
                        SCEuint, SCEubyte*);

extern void *TBrush_SphereData;

#endif /* guard */
