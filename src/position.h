/*------------------------------------------------------------------------------
    Spaceracer - A space racing and shooting game
    Copyright (C) 2008-2011  Antony Martin <martin(dot)antony(at)yahoo(dot)fr>
                             Colomban Wendling <ban(at)herbesfolles(dot)org>

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

#ifndef H_POSITION
#define H_POSITION

#include <SCE/utils/SCEUtils.h>

/* level of compression of the data */
enum {
    POS_COMP_0,                 /* none compression */
    POS_COMP_1,                 /* trying to compress angular velocity */
    POS_COMP_2,                 /* trying to compress both velocities */
    POS_COMP_3,                 /* velocities and rotation */
    POS_COMP_4                  /* all (including position) */
};

/* masks for informations */
enum {
    POS_MASK_AVEL = 1 << 0,
    POS_MASK_LVEL = 1 << 1,
    POS_MASK_ROT  = 1 << 2,
    POS_MASK_POS  = 1 << 3
};
/* all masks combined */
#define POS_MASK_ALL (POS_MASK_AVEL | POS_MASK_LVEL | \
                      POS_MASK_ROT | POS_MASK_POS)

/* for the price of 13 floats we have all the required data about any moving
   object. one byte is allowed to */
typedef struct {
    SCE_TVector3 pos, lvel, avel, scale;
    SCE_TQuaternion rot;        /* NOTE: perhaps 3 floats are enough? */
} Position;

void Pos_Init (Position*);
void Pos_Copy (Position*, Position*);
/* these just set pos and rot */
void Pos_SetFromMatrix4 (Position*, SCE_TMatrix4);
void Pos_SetFromMatrix4x3 (Position*, SCE_TMatrix4x3);
void Pos_SetToMatrix4 (Position*, SCE_TMatrix4);
void Pos_SetToMatrix4x3 (Position*, SCE_TMatrix4x3);

void Pos_Interpolate (Position*, Position*, float, Position*);

void Pos_ApplyVelocities (Position*, float);

void Pos_Difference (const Position*, const Position*, Position*);

size_t Pos_PackIntoBuffer (Position*, unsigned char*);
void Pos_UnpackFromBuffer (Position*, size_t, unsigned char*);

#endif /* guard */
