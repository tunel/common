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

#ifndef H_CHARACTER
#define H_CHARACTER

#include <SCE/utils/SCEUtils.h>
#include "physics.h"

typedef struct character Character;
struct character {
    PhyCharacter *pc;
    float radius;
    float height;
    float max_speed;                 /* max linear speed */
    float jump_impulse;
    float mass;
    float friction;
    float jump;
};

void Char_Init (Character*);
void Char_Clear (Character*);
Character* Char_New (void);
void Char_Free (Character*);

void Char_SetDimensions (Character*, float, float);

void Char_SetMaximumSpeed (Character*, float);
float Char_GetMaximumSpeed (const Character*);
void Char_SetJumpImpulse (Character*, float);

void Char_SetMass (Character*, float);
void Char_SetFriction (Character*, float);
void Char_SetPosition (Character*, float, float, float);
void Char_SetPositionv (Character*, const SCE_TVector3);

int Char_Build (Character*);

Physics* Char_GetPhysics (Character*);
PhyCharacter* Char_GetPhyCharacter (Character*);

void Char_Jump (Character*, float);
void Char_Move (Character*, const SCE_TVector3);

int Char_OnGround (const Character*);

#endif /* guard */
