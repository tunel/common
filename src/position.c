/*------------------------------------------------------------------------------
    Spaceracer - A space racing and shooting game
    Copyright (C) 2008-2012  Antony Martin <martin(dot)antony(at)yahoo(dot)fr>
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

#include "position.h"

void Pos_Init (Position *pos)
{
    SCE_Vector3_Set (pos->pos, 0.0, 0.0, 0.0);
    SCE_Vector3_Set (pos->lvel, 0.0, 0.0, 0.0);
    SCE_Vector3_Set (pos->avel, 0.0, 0.0, 0.0);
    SCE_Vector3_Set (pos->scale, 1.0, 1.0, 1.0);
    SCE_Quaternion_Identity (pos->rot);
}
void Pos_Copy (Position *dst, Position *src)
{
    SCE_Vector3_Copy (dst->pos, src->pos);
    SCE_Vector3_Copy (dst->lvel, src->lvel);
    SCE_Vector3_Copy (dst->avel, src->avel);
    SCE_Vector3_Copy (dst->scale, src->scale);
    SCE_Quaternion_Copy (dst->rot, src->rot);
}

void Pos_SetFromMatrix4 (Position *pos, SCE_TMatrix4 m)
{
    SCE_TVector3 invscale;
    SCE_TMatrix4 mat;
    SCE_Matrix4_Copy (mat, m);
    SCE_Matrix4_GetScale (mat, pos->scale);
    SCE_Vector3_Operator1v (invscale, = 1.0 /, pos->scale);
    SCE_Matrix4_GetTranslation (mat, pos->pos);
    SCE_Matrix4_MulScalev (mat, invscale);
    SCE_Matrix4_ToQuaternion (mat, pos->rot);
}
void Pos_SetFromMatrix4x3 (Position *pos, SCE_TMatrix4x3 m)
{
    SCE_TVector3 invscale;
    SCE_TMatrix4x3 mat;
    SCE_Matrix4x3_Copy (mat, m);
    SCE_Matrix4x3_GetScale (mat, pos->scale);
    SCE_Vector3_Operator1v (invscale, = 1.0 /, pos->scale);
    SCE_Matrix4x3_GetTranslation (mat, pos->pos);
    SCE_Matrix4x3_MulScalev (mat, invscale);
    SCE_Matrix4x3_ToQuaternion (mat, pos->rot);
}
void Pos_SetToMatrix4 (Position *pos, SCE_TMatrix4 m)
{
    SCE_Matrix4_FromQuaternion (m, pos->rot);
    SCE_Matrix4_SetTranslation (m, pos->pos);
    SCE_Matrix4_MulScalev (m, pos->scale);
}
void Pos_SetToMatrix4x3 (Position *pos, SCE_TMatrix4x3 m)
{
    SCE_Matrix4x3_FromQuaternion (m, pos->rot);
    SCE_Matrix4x3_SetTranslation (m, pos->pos);
    SCE_Matrix4x3_MulScalev (m, pos->scale);
}


void Pos_Interpolate (Position *a, Position *b, float w, Position *r)
{
    SCE_Quaternion_Linear (a->rot, b->rot, w, r->rot);
    SCE_Vector3_Interpolate (a->pos, b->pos, w, r->pos);
    SCE_Vector3_Interpolate (a->lvel, b->lvel, w, r->lvel);
    SCE_Vector3_Interpolate (a->avel, b->avel, w, r->avel);
    SCE_Vector3_Interpolate (a->scale, b->scale, w, r->scale);
}


/**
 * \brief Apply the velocities of a Position object
 * \param secs time elapsed since the last update
 */
void Pos_ApplyVelocities (Position *pos, float secs)
{
    SCE_TQuaternion q;
    SCE_TVector3 axis;
    float angle;
    angle = SCE_Vector3_Length (pos->avel);
    if (angle > 0.0001) {
        SCE_Vector3_Operator2 (axis, =, pos->avel, /, angle);
        SCE_Quaternion_Rotatev (q, angle * secs, axis);
        SCE_Quaternion_MulCopy (pos->rot, q);
    }
    SCE_Quaternion_RotateV3 (pos->rot, pos->lvel, axis);
    SCE_Vector3_Operator2 (pos->pos, +=, axis, *, secs);
}


void Pos_Difference (const Position *a, const Position *b, Position *d)
{
    SCE_Vector3_Operator2v (d->pos, =, a->pos, -, b->pos);
    SCE_Vector3_Operator2v (d->lvel, =, a->lvel, -, b->lvel);
    SCE_Vector3_Operator2v (d->avel, =, a->avel, -, b->avel);
    SCE_Vector3_Operator2v (d->scale, =, a->scale, -, b->scale);
    d->rot[0] = a->rot[0] - b->rot[0];
    d->rot[1] = a->rot[1] - b->rot[1];
    d->rot[2] = a->rot[2] - b->rot[2];
    d->rot[3] = a->rot[3] - b->rot[3];
}


static unsigned char*
Pos_PackFloats (float *v, int num, unsigned char ne, unsigned char nm,
                int *offset, unsigned char *buf)
{
    int i;
    for (i = 0; i < num; i++)
        SCE_Encode_Float (v[i], SCE_TRUE, ne, nm, offset, &buf);
    return buf;
}
/**
 * \brief Makes a sendable buffer from an object
 */
size_t Pos_PackIntoBuffer (Position *pos, unsigned char *buf)
{
    unsigned char ne = 8, nm = 23;
    int offset = 0;
    buf = Pos_PackFloats (pos->pos, 3, ne, nm, &offset, buf);
    buf = Pos_PackFloats (pos->rot, 4, ne, nm, &offset, buf);
    buf = Pos_PackFloats (pos->lvel, 3, ne, nm, &offset, buf);
    buf = Pos_PackFloats (pos->avel, 3, ne, nm, &offset, buf);
          Pos_PackFloats (pos->scale, 3, ne, nm, &offset, buf);

    return (3 + 4 + 3 + 3 + 3) * 4; /* 4 bytes for each float */
}


static unsigned char*
Pos_UnpackFloats (float *v, int num, unsigned char ne, unsigned char nm,
                  int *offset, unsigned char *buf)
{
    int i;
    for (i = 0; i < num; i++)
        v[i] = SCE_Decode_Float (&buf, SCE_TRUE, ne, nm, offset);
    return buf;
}
/**
 * \brief Makes an object from a received buffer from network
 */
void Pos_UnpackFromBuffer (Position *pos, size_t size, unsigned char *buf)
{
    unsigned char ne = 8, nm = 23;
    int offset = 0;
    (void)size;
    buf = Pos_UnpackFloats (pos->pos, 3, ne, nm, &offset, buf);
    buf = Pos_UnpackFloats (pos->rot, 4, ne, nm, &offset, buf);
    buf = Pos_UnpackFloats (pos->lvel, 3, ne, nm, &offset, buf);
    buf = Pos_UnpackFloats (pos->avel, 3, ne, nm, &offset, buf);
          Pos_UnpackFloats (pos->scale, 3, ne, nm, &offset, buf);
}
