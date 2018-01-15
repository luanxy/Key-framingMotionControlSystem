#include "stdafx.h"

#include <comdef.h>

// standard
#include <assert.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

// glut
#include <GL/glut.h>

// source
#include <math/vec3.h>
#include <model.h>
using namespace std;
//================================
// global variables
//================================
#define RESOURCE_DIR   "data/"
#define PI 3.14159265358979323846

// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;

// frame index
static int g_frameIndex = 0;

// frame number
static int frameNumber = 5;

// model
Model g_model1;

// time
static GLfloat t = 0;

// input and spline mode
static int mode = 1;

// matrix
static GLfloat qMatrix[16] = { 0 };
static GLfloat resultMatrix[7] = { 0 };

// input keyframe
static GLfloat eulerAngleArray[5][6] = { { 90,0,0,5,5,-10 },{ 0,90,90,-5,5,-10 },{ 0,0,90,-5,-5,-10 },{ 90,90,0,10,-5,-10 },
{ 90,0,0,5,5,-5 } };
static GLfloat quaternionArray[5][7] = { { 1,0,0,0,5,5,-10 },{ 0,1,0,0,-5,5,-10 },{ 0,0,1,0,-5,-5,-10 },{ 0,0,0,1,10,-5,-10 },
{ 1,0,0,0,5,5,-5 } };

// spline matrix
static GLfloat CatmullRomSpline[16] = {-0.5f, 2 - 0.5f, 0.5f - 2, 0.5f, 1, 0.5f - 3, 3 - 2 * 0.5f, -0.5f, -0.5f, 0, 0.5f, 0, 
0, 1, 0, 0};
static GLfloat B_Spline[16] = {-1.0f / 6, 3.0f / 6, -3.0f / 6, 1.0 / 6, 3.0f / 6, -6.0f / 6, 3.0f / 6, 0.0f/ 6,  -3.0f / 6,
0.0f / 6, 3.0f / 6, 0.0f / 6, 1.0f / 6, 4.0f / 6, 1.0f / 6 ,0.0f / 6 };


//================================
// matrix methods
//================================
// nomalise quaternion
void nomalise(GLfloat quaternion[7]) {
	GLfloat totalLength = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] +
		quaternion[3] * quaternion[3];
	if (totalLength != 0 && (fabs(totalLength - 1.0f) > 0.00001f)) {
		GLfloat nomaliseLength = sqrt(totalLength);
		for (int i = 0; i <= 3; i++) {
			quaternion[i] /= nomaliseLength;
		}
	}
}

// transform euler to quaternion
void eulerToQuaternion(GLfloat euler[7]) {
	GLfloat psi = ( (euler[0] / 180) * PI ) / 2.0f;
	GLfloat theta =( (euler[1] / 180) * PI )/ 2.0f;
	GLfloat phi =( (euler[2] / 180) * PI )/ 2.0f;
	
	euler[6] = euler[5];
	euler[5] = euler[4];
	euler[4] = euler[3];
	euler[0] = cos(psi) * cos(theta) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
	euler[1] = sin(psi) * cos(theta) * cos(phi) - cos(psi) * sin(theta) * sin(phi);
	euler[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * cos(theta) * sin(phi);
	euler[3] = cos(psi) * sin(theta) * sin(phi) + sin(psi) * sin(theta) * cos(phi);
	
}

// calculate quaternion matrix
void quaternionToMatrix(GLfloat quaternion[7], GLfloat matrix[16]) {
	GLfloat w = quaternion[0];
	GLfloat x = quaternion[1];
	GLfloat y = quaternion[2];
	GLfloat z = quaternion[3];

	matrix[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
	matrix[1] = 2.0f * x * y + 2.0f * w * z;
	matrix[2] = 2.0f * x * z - 2.0f * w * y;
	matrix[3] = 0.0f;
	matrix[4] = 2.0f * x * y - 2.0f * w * z;
	matrix[5] = 1.0f - 2.0f * x * x - 2.0f * z * z;
	matrix[6] = 2.0f * y * z + 2.0f * w * x;
	matrix[7] = 0.0f;
	matrix[8] = 2.0f * x * z + 2.0f * w * y;
	matrix[9] = 2.0f * y * z - 2.0f * w * x;
	matrix[10] = 1.0f - 2.0f * x * x - 2.0f * y * y;
	matrix[11] = 0.0f;
	matrix[12] = quaternion[4];
	matrix[13] = quaternion[5];
	matrix[14] = quaternion[6];
	matrix[15] = 1.0f;
}

// calculate T * M * G
GLfloat multiplyMatrix(GLfloat T[4], GLfloat M[16], GLfloat G[4]) {
	GLfloat MG[4] = {0};
	GLfloat result = 0;

	MG[0] = M[0] * G[0] + M[1] * G[1] + M[2] * G[2] + M[3] * G[3];
	MG[1] = M[4] * G[0] + M[5] * G[1] + M[6] * G[2] + M[7] * G[3];
	MG[2] = M[8] * G[0] + M[9] * G[1] + M[10] * G[2] + M[11] * G[3];
	MG[3] = M[12] * G[0] + M[13] * G[1] + M[14] * G[2] + M[15] * G[3];

	result = T[0] * MG[0] + T[1] * MG[1] + T[2] * MG[2] + T[3] * MG[3];
	return result;
}

// calculte euler interpolation
void interpolation(GLfloat keyFrame[5][6],GLfloat M[16]) {
	GLfloat T[4] = { t*t*t,t*t,t,1 };

	for (int i = 0; i<6; i++) {
		GLfloat G[4] = { keyFrame[g_frameIndex][i],keyFrame[(g_frameIndex + 1) % frameNumber][i],keyFrame[(g_frameIndex + 2) % frameNumber][i],
			keyFrame[(g_frameIndex + 3) % frameNumber][i] };
		resultMatrix[i] = multiplyMatrix(T, M, G);
	}
	eulerToQuaternion(resultMatrix);
	nomalise(resultMatrix);
	quaternionToMatrix(resultMatrix, qMatrix);
}

// calculte quaternion interpolation
void interpolation(GLfloat keyFrame[5][7], GLfloat M[16]) {
	GLfloat T[4] = { t*t*t,t*t,t,1 };

	for (int i = 0; i<7; i++) {
		GLfloat G[4] = { keyFrame[g_frameIndex][i],keyFrame[(g_frameIndex + 1) % frameNumber][i],keyFrame[(g_frameIndex + 2) % frameNumber][i],
			keyFrame[(g_frameIndex + 3) % frameNumber][i] };
		resultMatrix[i] = multiplyMatrix(T, M, G);
	}
	nomalise(resultMatrix);
	quaternionToMatrix(resultMatrix, qMatrix);
}

//================================
// init
//================================
void init( void ) {
	// init something before main loop...

	// load model
	g_model1.LoadModel( "data/cow.d" ); 

}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...
}

//================================
// render
//================================

void render( void ) {
	// clear color and depth buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glClearDepth ( 1.0 );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	
	// enable depth test
	glEnable( GL_DEPTH_TEST );
	glShadeModel(GL_SMOOTH);

	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	//choose mode
	if(mode == 1)
		interpolation(eulerAngleArray, CatmullRomSpline);
	else if(mode == 2)
		interpolation(quaternionArray, CatmullRomSpline);
	else if (mode == 3)
		interpolation(eulerAngleArray, B_Spline);
	else
		interpolation(quaternionArray, B_Spline);

	// draw model1
	glLoadMatrixf(qMatrix);
	glColor3f(0.1f, 0.2f, 0.3f);
	g_model1.DrawPoly();


	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void key_press( unsigned char key, int x, int y ) {
	switch (key) {
	//switch mode
	case '1':
		mode = 1;
		break;
	case '2':
		mode = 2;
		break;
	case '3':
		mode = 3;
		break;
	case '4':
		mode = 4;
		break;
	}
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;	
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix <------------------------------------------
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(100.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}

//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {
	
	// render
	glutPostRedisplay();

	// increase frame index
	t += 0.01;

	if (t >= 1) {
		t = 0;
		if (g_frameIndex < frameNumber - 1) {
			g_frameIndex++;
		}
		else {
			g_frameIndex = 0;
		}
	}

	// reset timer
	glutTimerFunc( 16, timer, 0 );
}

//================================
// main
//================================
int main( int argc, char** argv ) {

	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 800, 600 ); 
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( "Assignment 1" );

	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( key_press ); 
	glutTimerFunc( 16, timer, 0 );
	
	// main loop
	glutMainLoop();

	return 0;
}