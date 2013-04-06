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
#include "physics.h"
#include "character.h"

void Char_Init (Character *ch)
{
    ch->pc = NULL;
    ch->radius = 0.3f;
    ch->height = 1.2f;
    ch->max_speed = 1.0;
    ch->jump_impulse = 1.0;
    ch->mass = 1.0;
    ch->friction = 3.0;
    ch->jump = 0.0;
    ch->rotation = 0.0;
}
void Char_Clear (Character *ch)
{
    Phy_FreeCharacter (ch->pc);
}
Character* Char_New (void)
{
    Character *ch = NULL;
    if (!(ch = SCE_malloc (sizeof *ch)))
        goto fail;
    Char_Init (ch);
    if (!(ch->pc = Phy_NewCharacter ()))
        goto fail;

    return ch;
fail:
    SCEE_LogSrc ();
    return NULL;
}
void Char_Free (Character *ch)
{
    if (ch) {
        Char_Clear (ch);
        SCE_free (ch);
    }
}

void Char_SetDimensions (Character *ch, float r, float h)
{
    ch->radius = r;
    ch->height = h;
    Phy_SetCharacterDimensions (ch->pc, r, h);
}

void Char_SetMaximumSpeed (Character *ch, float speed)
{
    ch->max_speed = speed;
}
float Char_GetMaximumSpeed (const Character *ch)
{
    return ch->max_speed;
}
void Char_SetJumpImpulse (Character *ch, float f)
{
    ch->jump_impulse = f;
}

void Char_SetMass (Character *ch, float mass)
{
    ch->mass = mass;
}
void Char_SetFriction (Character *ch, float friction)
{
    ch->friction = friction;
}
void Char_SetPosition (Character *ch, float x, float y, float z)
{
    Phy_SetPosition (Char_GetPhysics (ch), x, y, z);
}
void Char_SetPositionv (Character *ch, const SCE_TVector3 v)
{
    Phy_SetPositionv (Char_GetPhysics (ch), v);
}

int Char_Build (Character *ch)
{
    Physics *phy = NULL;

    phy = Phy_GetCharacterPhysics (ch->pc);
    Phy_SetMass (phy, ch->mass);
    Phy_SetFriction (phy, ch->friction);

    if (Phy_BuildCharacter (ch->pc) < 0)
        goto fail;

    return SCE_OK;
fail:
    SCEE_LogSrc ();
    return SCE_ERROR;
}

Physics* Char_GetPhysics (Character *ch)
{
    return Phy_GetCharacterPhysics (ch->pc);
}
PhyCharacter* Char_GetPhyCharacter (Character *ch)
{
    return ch->pc;
}

void Char_SetRotation (Character *ch, float angle)
{
    ch->rotation = angle;
}

void Char_Jump (Character *ch, float factor)
{
    ch->jump = factor;
}
/**
 * \brief Move a character
 * \param ch a character
 * \param move moving factors, vector length must be between 0 and 1
 */
void Char_Move (Character *ch, const SCE_TVector3 move)
{
    SCE_TVector3 forward_vec = {0.0, 1.0, 0.0};
    SCE_TVector3 side_vec = {1.0, 0.0, 0.0};
    SCE_TVector3 v;
    float a = ch->rotation;

    SCE_Vector3_RotateZ (forward_vec, cos (a), sin (a));
    SCE_Vector3_RotateZ (side_vec, cos (a), sin (a));

    SCE_Vector3_Operator1v (v, = move[1] *, forward_vec);
    SCE_Vector3_Operator1v (v, += move[0] *, side_vec);

    SCE_Vector3_Operator1 (v, *=, ch->max_speed);

    v[2] += ch->jump * ch->jump_impulse;
    Phy_SetCharacterVelocityv (ch->pc, v);
    if (Char_OnGround (ch))
        ch->jump = 0.0;
}

int Char_OnGround (const Character *ch)
{
    return Phy_IsCharacterOnGround (ch->pc);
}
