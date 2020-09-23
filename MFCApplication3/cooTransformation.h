#pragma once

typedef struct
{
	float Rx;
	float Ry;
	float Rz;
	float x;
	float y;
	float z;

}PoseTydeDef;

void Transformate(PoseTydeDef A, PoseTydeDef B, PoseTydeDef* C);

void cooMulti(PoseTydeDef A, PoseTydeDef B, PoseTydeDef* C);