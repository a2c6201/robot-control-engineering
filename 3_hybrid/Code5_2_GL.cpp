#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <GLUT/glut.h>
#include <GL/gl_screenshot.h>
using namespace std;

int WindowPositionX = 200;  //��������E�B���h�E�ʒu��X���W
int WindowPositionY = 100;  //��������E�B���h�E�ʒu��Y���W
int WindowWidth = 640;    //��������E�B���h�E�̕�
int WindowHeight = 480;    //��������E�B���h�E�̍���
char WindowTitle[] = "Motion of Robot Arm";  //�E�B���h�E�̃^�C�g��

static GLfloat floor_planar[4];
static GLfloat floor_s = 50.0f;
//static GLfloat floor_s = 50.0f;
static GLfloat pM[16];
//static GLfloat lightpos[4] = { -110, -100, 50, 1 };
static GLfloat lightpos[4] = { 20, 20, 100, 1 };
const int origin_x = -20;
const int origin_y = 10;

typedef struct _QUADS_VERTEX{
	GLfloat v0[3];
	GLfloat v1[3];
	GLfloat v2[3];
	GLfloat v3[3];
}QUADS_VERTEX;
static QUADS_VERTEX floor_v = {
	{ floor_s, floor_s, -1.0f },
	{ -floor_s, floor_s, -1.0f },
	{ -floor_s, -floor_s, -1.0f },
	{ floor_s, -floor_s, -1.0f },
};

#define _BITMAP 0
const int dK = 5;
int tn = 0;
//double dt = 0.05;
gl_screenshot gs; //bmp�t�@�C���̏o��

struct {
	double x, y, z;
	double vx, vy, vz;
}p[100];
int pn = 0;
double ax = 0.0, ay = 0.0, az = -4.0;
double vx = 5.0, vy = 5.0, vz = 20.0;
double hanpatu = 0.9;

GLUquadricObj *quadric = NULL;

//--------------- Robot Arm --------------------------
const double L1 = 0.3;
const double L2 = 0.3;
const double Lg1 = 0.15;
const double Lg2 = 0.15;
const double m1 = 0.5;
const double m2 = 0.5;
const double I1 = 5.4e-3;
const double I2 = 5.4e-3;
const double mL = 5.0;
const double xvh = 1.0;
const double yvh = 1.0;
const double g = 9.806199;
const double pi = 3.141592;
const double h = 0.001;

double t, th1[801], th2[801], thv1[801], thv2[801];
double X1[801], Y1[801], X2[801], Y2[801];
void zp1(void);
//----------------------------------------------------


int k;
//----------------------------------------------------
// ���������̒�`
//----------------------------------------------------
struct MaterialStruct {
	GLfloat ambient[4];
	GLfloat diffuse[4];
	GLfloat specular[4];
	GLfloat shininess;
};
//jade(�Ő�)
MaterialStruct ms_jade = {
	{ 0.135, 0.2225, 0.1575, 1.0 },
	{ 0.54, 0.89, 0.63, 1.0 },
	{ 0.316228, 0.316228, 0.316228, 1.0 },
	12.8 };
//ruby(���r�[)
MaterialStruct ms_ruby = {
	{ 0.1745, 0.01175, 0.01175, 1.0 },
	{ 0.61424, 0.04136, 0.04136, 1.0 },
	{ 0.727811, 0.626959, 0.626959, 1.0 },
	76.8 };
//----------------------------------------------------
// �֐��v���g�^�C�v�i��ɌĂяo���֐����ƈ����̐錾�j
//----------------------------------------------------
void Initialize(void);
void Display(void);
//void Idle();
void timer(int value);
void Keyboard(unsigned char key, int x, int y);
void Ground(void);  //��n�̕`��

void findPlane(GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3]);
void shadowMatrix(GLfloat *m, GLfloat plane[4], GLfloat light[4]);
void DrawFloor(bool bTexture);
void DrawShadow(void);
void DrawStructure(bool);
//----------------------------------------------------
// ���C���֐�
//----------------------------------------------------
int main(int argc, char *argv[]){
	srand((unsigned)time(NULL));
#if _BITMAP
	_mkdir("bitmap"); //bmp�t�@�C���ۑ��p�̃t�H���_�̍쐬
#endif

	int i, n;
	double A1, A2, A3, B1, B2;
	double z1, z2, w1, w2;
	double z21, z22, w21, w22;
	double z31, z32, w31, w32;
	double z41, z42, w41, w42;
	double a11, a12, a22, u1, u2, LL;
	double k11, k12, k21, k22, k31, k32, k41, k42;
	double L11, L12, L21, L22, L31, L32, L41, L42;
	double tau1, tau2;   //���[�^�g���N(�����)
	double tauL1, tauL2; //���׃g���N

	double FL, FLx, FLy; //�O��
	double Xp, Yp; // ���݂̈ʒu
	double Xv, Yv; // ���݂̑��x
	double zd1, zd2; //�ڕW�p�x
	double Sn1, Sn2; //�񊱏���̔}�̕ϐ�
	double Xd, Yd; //�ڕW�ʒu
	double dx, dy, dp; //�ڕW�ʒu�̔����l
	double dzd1, dzd2; //�ڕW�p�x�̔����l
	double Sp1, Sp2, Sv1, Sv2, Sf1, Sf2; //Hybrid�̕ϐ�
	double tauP1, tauP2; //�ʒu����̃g���N
	double tauML1, tauML2;//�͐���̃g���N(P)
	double tauMS1, tauMS2;//�͐���̃g���N(I)
	double tauF1, tauF2;//�͐���̃g���N
	double Fd;      //�ǉ��t�̖͂ڕW�l
	double FLA = 0; //�ǂ���̔��͂̕��ϒl
	double zw; //�A�[����[�̈ړ������p�x(Y�����甽���v��肪��)
	double dxv, dyv, dv, dwd1, dwd2; // ���x�E�p���x�̔����l
	double Ssf1 = 0, Ssf2 = 0; // �͂̎��Ԑϕ�
	double DJ;
	int nh=0;

	A1 = m1*Lg1*Lg1 + I1 + m2*L1*L1 + mL*L1*L1;
	A2 = I2 + m2*Lg2*Lg2 + mL*L2*L2;
	A3 = (m2*Lg2 + mL*L2)*L1;
	B1 = (m1*Lg1 + m2*L1 + mL*L1)*g;
	B2 = (m2*Lg2 + mL*L2)*g;

	// ----- �����l�ݒ� -----
	t = 0.0;
	z1 = 90.0*pi / 180.0;
	z2 = -90.0*pi / 180.0;
	w1 = 0;	w2 = 0;
	Yp = L1;
	Xv = 0; Yv = 0;

	zd1 =  pi / 2.0;
	zd2 = -pi / 2.0;

	// 800 ms�Ԃ̃��[�v
	for (i = 0; i <= 800; i++){
		t = t + h;

		Xp = L1*cos(z1) + L2*cos(z1 + z2);
		Yp = L1*sin(z1) + L2*sin(z1 + z2);
		Xv = (-L1*sin(z1) - L2*sin(z1 + z2))*w1
		   + (-L2*sin(z1 + z2))*w2;
		Yv = (L1*cos(z1) + L2*cos(z1 + z2))*w1
		   + (L2*cos(z1 + z2))*w2;

		// ----- �O�� -----
		if (Yp > -2.0*Xp + 1.1){

			FL = -100000.0 * (Yp + 2.0*Xp - 1.1)*sin(26.56505118*pi / 180.0);

			FLx = FL * cos(26.56505118*pi / 180.0);
			FLy = FL * sin(26.56505118*pi / 180.0);
		}
		else{
			FLx = 0.0;
			FLy = 0.0;
			FL = 0.0;
		}

		// ----- ����� -----
		if (i >= 0 && i <= 200)      { Xd = 0.3; Yd = 0.1; }
		else if (i >= 200 && i < 400){ Xd = 0.5; Yd = 0.1; }
		else if (i >= 400 && i < 600){ Xd = 0.4; Yd = 0.3; }
		else if (i >= 600 && i < 800){ Xd = 0.3; Yd = 0.3; }
		if (nh == 199) nh = 0;
		else           nh++;
		dx = 1.0 / (200.0 - (double)nh)*(Xd - Xp);
		dy = 1.0 / (200.0 - (double)nh)*(Yd - Yp);


		if (Yp > -2.0*Xp + 1.1 && i<600){ //�ʒu�Ɨ͂̃n�C�u���b�g����
			zw = -atan(dx / dy);

			dp = sqrt(dx*dx + dy*dy);
			Sp1 = -sin(zw) * dp;
			Sp2 = cos(zw) * dp;
			DJ = L1*L2*sin(z2);
			dzd1 = L2*cos(z1 + z2) / DJ * Sp1 + L2 *sin(z1 + z2) / DJ * Sp2;
			dzd2 = (-L1*cos(z1) - L2*cos(z1 + z2)) / DJ * Sp1 + (-L1*sin(z1) - L2*sin(z1 + z2)) / DJ * Sp2;
			zd1 += dzd1;
			zd2 += dzd2;
			tauP1 = 5000.0*(zd1 - z1) + 150.0*(-w1);
			tauP2 = 5000.0*(zd2 - z2) + 150.0*(-w2);

			Fd = 200.0; //�ǂ����t�����; 200 N�ɐݒ�
			Sf1 =  cos(zw) * (Fd - (-FL));
			Sf2 = sin(zw) * (Fd - (-FL));
			tauML1 = (-L1*sin(z1) - L2*sin(z1 + z2)) * Sf1 + (L1*cos(z1) + L2*cos(z1 + z2)) * Sf2;
			tauML2 = (-L2*sin(z1 + z2)) * Sf1              + (L2*cos(z1 + z2)) * Sf2;
			Ssf1 += cos(zw) * (Fd - (-FL));
			Ssf2 += sin(zw) * (Fd - (-FL));
			tauMS1 = (-L1*sin(z1) - L2*sin(z1 + z2)) * Ssf1 + (L1*cos(z1) + L2*cos(z1 + z2)) * Ssf2;
			tauMS2 = (-L2*sin(z1 + z2)) * Ssf1 + (L2*cos(z1 + z2)) * Ssf2;
			tauF1 = 50.0*tauML1 + 5.0*tauMS1;
			tauF2 = 50.0*tauML2 + 5.0*tauMS2;

			tau1 = tauP1 + tauF1;
			tau2 = tauP2 + tauF2;
		}
		else{ //�ʒu����̂�
			DJ = L1*L2*sin(z2);
			dzd1 = L2 / DJ*cos(z1 + z2)*dx + L2 / DJ*sin(z1 + z2)*dy;
			dzd2 = (-L1*cos(z1) - L2*cos(z1 + z2)) / DJ*dx + (-L1*sin(z1) - L2*sin(z1 + z2)) / DJ*dy;
			zd1 += dzd1;
			zd2 += dzd2;

			tau1 = 25000.0*(zd1 - z1) + 250.0*(-w1);
			tau2 = 25000.0*(zd2 - z2) + 250.0*(-w2);
		}

		// ----- �����Q�E�N�b�^�@�@��1�i�K -----
		k11 = h*(w1);
		k12 = h*(w2);
		LL = A1*A2 - A3*A3*cos(z2)*cos(z2);
		a11 = A2 / LL;
		a12 = -(A2 + A3*cos(z2)) / LL;
		a22 = (A1 + A2 + 2.0 * A3*cos(z2)) / LL;
		tauL1 = (-L1*sin(z1) - L2*sin(z1 + z2))*FLx
			+ (L1*cos(z1) + L2*cos(z1 + z2))*FLy;
		tauL2 = (-L2*sin(z1 + z2))*FLx
			+ (L2*cos(z1 + z2))*FLy;
		u1 = tau1 + tauL1
			+ A3*(2.0*w1*w2 + w2*w2)*sin(z2)
			- B1*cos(z1) - B2*cos(z1 + z2);
		u2 = tau2 + tauL2
			- A3*w1*w1*sin(z2)
			- B2*cos(z1 + z2);
		L11 = h*(a11*u1 + a12*u2);
		L12 = h*(a12*u1 + a22*u2);

		// ----- �����Q�E�N�b�^�@�@��2�i�K -----
		z21 = z1 + k11 / 2.0;
		z22 = z2 + k12 / 2.0;
		w21 = w1 + L11 / 2.0;
		w22 = w2 + L12 / 2.0;
		k21 = h*(w21);
		k22 = h*(w22);
		LL = A1*A2
			- A3*A3*cos(z22)*cos(z22);
		a11 = A2 / LL;
		a12 = -(A2 + A3*cos(z22)) / LL;
		a22 = (A1 + A2 + 2.0 * A3*cos(z22)) / LL;
		tauL1 = (-L1*sin(z21) - L2*sin(z21 + z22))*FLx
			+ (L1*cos(z21) + L2*cos(z21 + z22))*FLy;
		tauL2 = (-L2*sin(z21 + z22))*FLx
			+ (L2*cos(z21 + z22))*FLy;
		u1 = tau1 + tauL1
			+ A3*(2.0*w21*w22 + w22*w22)*sin(z22)
			- B1*cos(z21) - B2*cos(z21 + z22);
		u2 = tau2 + tauL2
			- A3*w21*w21*sin(z22)
			- B2*cos(z21 + z22);
		L21 = h*(a11*u1 + a12*u2);
		L22 = h*(a12*u1 + a22*u2);

		// ----- �����Q�E�N�b�^�@�@��3�i�K -----
		z31 = z1 + k21 / 2.0;
		z32 = z2 + k22 / 2.0;
		w31 = w1 + L21 / 2.0;
		w32 = w2 + L22 / 2.0;
		k31 = h*w31;
		k32 = h*w32;
		LL = A1*A2 - A3*A3*cos(z32)*cos(z32);
		a11 = A2 / LL;
		a12 = -(A2 + A3*cos(z32)) / LL;
		a22 = (A1 + A2 + 2.0 * A3*cos(z32)) / LL;
		tauL1 = (-L1*sin(z31) - L2*sin(z31 + z32))*FLx
			+ (L1*cos(z31) + L2*cos(z31 + z32))*FLy;
		tauL2 = (-L2*sin(z31 + z32))*FLx
			+ (L2*cos(z31 + z32))*FLy;
		u1 = tau1 + tauL1
			+ A3*(2.0*w31*w32 + w32*w32)*sin(z32)
			- B1*cos(z31) - B2*cos(z31 + z32);
		u2 = tau2 + tauL2
			- A3*w31*w31*sin(z32)
			- B2*cos(z31 + z32);
		L31 = h*(a11*u1 + a12*u2);
		L32 = h*(a12*u1 + a22*u2);

		// ----- �����Q�E�N�b�^�@�@��4�i�K -----
		z41 = z1 + k31;
		z42 = z2 + k32;
		w41 = w1 + L31;
		w42 = w2 + L32;
		k41 = h*w41;
		k42 = h*w42;
		LL = A1*A2
			- A3*A3*cos(z42)*cos(z42);
		a11 = A2 / LL;
		a12 = -(A2 + A3*cos(z42)) / LL;
		a22 = (A1 + A2 + 2.0 * A3*cos(z42)) / LL;
		tauL1 = (-L1*sin(z41) - L2*sin(z41 + z42))*FLx
			+ (L1*cos(z41) + L2*cos(z41 + z42))*FLy;
		tauL2 = (-L2*sin(z41 + z42))*FLx
			+ (L2*cos(z41 + z42))*FLy;
		u1 = tau1 + tauL1
			+ A3*(2.0*w41*w42 + w42*w42)*sin(z42)
			- B1*cos(z41) - B2*cos(z41 + z42);
		u2 = tau2 + tauL2
			- A3*w41*w41*sin(z42)
			- B2*cos(z41 + z42);
		L41 = h*(a11*u1 + a12*u2);
		L42 = h*(a12*u1 + a22*u2);

		// ----- �����Q�E�N�b�^�@�@�S�i���� -----
		z1 = z1 + (k11 + 2.0*k21 + 2.0*k31 + k41) / 6.0;
		z2 = z2 + (k12 + 2.0*k22 + 2.0*k32 + k42) / 6.0;
		w1 = w1 + (L11 + 2.0*L21 + 2.0*L31 + L41) / 6.0;
		w2 = w2 + (L12 + 2.0*L22 + 2.0*L32 + L42) / 6.0;

		th1[i] = z1;  th2[i] = z2;
		thv1[i] = w1; thv2[i] = w2;
		X1[i] = L1*cos(z1);
		Y1[i] = L1*sin(z1);
		X2[i] = L1*cos(z1) + L2*cos(z1 + z2);
		Y2[i] = L1*sin(z1) + L2*sin(z1 + z2);

		if(i>=400 && i<600) FLA += FL/200.0;

		if (i % 10 == 0){
			printf("i=%3d z1=%7.2f z2=%7.2f   "
			, i, th1[i] * 180.0 / pi, th2[i] * 180.0 / pi);
			printf("X=%6.3f Y=%6.3f   ", X2[i], Y2[i]);
			printf("F=%7.3f\n", -FL);
		}
		if(i==800) printf("FL_average=%7.3f\n", FLA);
	}

	glutInit(&argc, argv);//���̏�����
	glutInitWindowPosition(WindowPositionX, WindowPositionY);//�E�B���h�E�̈ʒu�̎w��
	glutInitWindowSize(WindowWidth, WindowHeight); //�E�B���h�E�T�C�Y�̎w��
	//	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);//�f�B�X�v���C���[�h�̎w��
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL);//�f�B�X�v���C���[�h�̎w�� [�X�e���V����ǉ����Ȃ���΂Ȃ�Ȃ�]
	glutCreateWindow(WindowTitle);  //�E�B���h�E�̍쐬
	glutDisplayFunc(Display); //�`�掞�ɌĂяo�����֐����w�肷��i�֐����FDisplay�j
	glutKeyboardFunc(Keyboard);//�L�[�{�[�h���͎��ɌĂяo�����֐����w�肷��i�֐����FKeyboard�j
//	glutIdleFunc(Idle);       //�v���O�����A�C�h����Ԏ��ɌĂяo�����֐�
	glutTimerFunc(0,timer,50);       //�v���O�����A�C�h����Ԏ��ɌĂяo�����֐�

	quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL);
	gluQuadricNormals(quadric, GLU_SMOOTH);

	Initialize(); //�����ݒ�̊֐����Ăяo��
	glutMainLoop();

	gluDeleteQuadric(quadric);
	return 0;
}

//----------------------------------------------------
// �����ݒ�̊֐�
//----------------------------------------------------
void Initialize(void){
	glClearColor(1.0, 1.0, 1.0, 1.0); //�w�i�F
	glEnable(GL_DEPTH_TEST); //�f�v�X�o�b�t�@���g�p�FglutInitDisplayMode() �� GLUT_DEPTH ���w�肷��
	glDepthFunc(GL_LEQUAL);
	glClearDepth(1.0);

	findPlane(floor_planar,
		floor_v.v0,
		floor_v.v1,
		floor_v.v2);

	//�����ϊ��s��̐ݒ�------------------------------
	glMatrixMode(GL_PROJECTION);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	gluPerspective(30.0, (double)WindowWidth / (double)WindowHeight, 0.1, 1000.0); //�������e�@�̎��̐�gluPerspactive(th, w/h, near, far);

}
//----------------------------------------------------
// �`��̊֐�
//----------------------------------------------------
void Display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	//���_�̐ݒ�------------------------------
	gluLookAt(
		0.0, -1.0, 300.0, // ���_�̈ʒux,y,z;
		0.0, 0.0, 0.0,   // ���E�̒��S�ʒu�̎Q�Ɠ_���Wx,y,z
		0.0, 0.0, 1.0);  //���E�̏�����̃x�N�g��x,y,z
	//----------------------------------------

	//���f���r���[�ϊ��s��̐ݒ�--------------------------
	glMatrixMode(GL_MODELVIEW);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	glViewport(0, 0, WindowWidth, WindowHeight);
	//----------------------------------------------

	//�X�e���V���o�b�t�@�N���A�l�̐ݒ�--------------------------
	glClearStencil(0);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	//----------------------------------------

	// ���ʎˉe�s��̎Z�o--------------------------
	shadowMatrix(pM, floor_planar, lightpos);
	//--------------------------

	// ����ON-----------------------------
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
	//-----------------------------------

	glPushMatrix();
	DrawStructure(false);
	DrawShadow();
	glPopMatrix();

	glDisable(GL_AUTO_NORMAL);
	glDisable(GL_NORMALIZE);

	//�A�eOFF-----------------------------
	glDisable(GL_LIGHTING);
	//-----------------------------------

	Ground();

	k+=dK;
	if (k>800) k = 0;


#if _BITMAP
	if (tn % 40 == 0 && tn<801){
		ostringstream fname;
		int tt = tn + 10000;
		fname << "bitmap/" << tt << ".bmp";//�o�̓t�@�C����
		string name = fname.str();
		gs.screenshot(name.c_str(), 24);
	}
	tn++;
#endif

	glutSwapBuffers(); //glutInitDisplayMode(GLUT_DOUBLE)�Ń_�u���o�b�t�@�����O�𗘗p��
}
//----------------------------------------------------
// ���̂̕`��
//----------------------------------------------------
void DrawStructure(bool flag){
	double angle1, angle2;

	p[0].x = origin_x;
	p[0].y = origin_y;
	p[0].z = 10.0;

	angle1 = th1[k] * 180.0 / 3.14156 - 90.0;
	angle2 = th2[k] * 180.0 / 3.14156 + 90.0 + (th1[k]*180/3.14156-90.0);

	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
	glTranslated(p[0].x, p[0].y, p[0].z);//���s�ړ��l�̐ݒ�
	glutSolidSphere(4.0, 20, 20);//�����F(���a, Z���܂��̕�����, Z���ɉ�����������)
	glPopMatrix();

	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	glTranslated(p[0].x, p[0].y, p[0].z);//���s�ړ��l�̐ݒ�
	glRotated(angle1, 0, 0, 1);
	glRotated(270.0, 1, 0, 0);
//	glutSolidSphere(4.0, 20, 20);//�����F(���a, Z���܂��̕�����, Z���ɉ�����������)
//	glutSolidCone(5.0, 10.0, 20, 20);
	gluCylinder(quadric, 3.0, 3.0, 100*L1, 20, 15);
//	gluCylinder(quadric, 1.0, 1.0, 50.0, 20, 15);
	glPopMatrix();



	p[1].x = X1[k] * 100.0 + origin_x;
	p[1].y = Y1[k] * 100.0 + origin_y;
	p[1].z = 10.0;
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
	glTranslated(p[1].x, p[1].y, p[1].z);//���s�ړ��l�̐ݒ�
	glutSolidSphere(4.0, 20, 20);//�����F(���a, Z���܂��̕�����, Z���ɉ�����������)
	glPopMatrix();

	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, ms_jade.ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_jade.diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, ms_jade.specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &ms_jade.shininess);
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	glTranslated(p[1].x, p[1].y, p[1].z);//���s�ړ��l�̐ݒ�
	glRotated(angle2, 0, 0, 1);
	glRotated(90.0, 0, 1, 0);
	gluCylinder(quadric, 3.0, 3.0, 100*L2, 20, 15);
	glPopMatrix();


	p[2].x = X2[k] * 100.0 + origin_x;
	p[2].y = Y2[k] * 100.0 + origin_y;
	p[2].z = 10.0;
	if (!flag || p[2].z >0){
		glPushMatrix();
		glMaterialfv(GL_FRONT, GL_AMBIENT, ms_ruby.ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, ms_ruby.diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, ms_ruby.specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, &ms_ruby.shininess);
		glTranslated(p[2].x, p[2].y, p[2].z);//���s�ړ��l�̐ݒ�
		glutSolidSphere(4.0, 20, 20);//�����F(���a, Z���܂��̕�����, Z���ɉ�����������)
		glPopMatrix();
	}
}

//----------------------------------------------------
// ��n�̕`��
//----------------------------------------------------
void Ground(void) {
	double ground_max_x = 270.0;
	double ground_max_y = 270.0;
	glColor3d(0.8, 0.8, 0.8);  // ��n�̐F
	glBegin(GL_LINES);
	for (double ly = -ground_max_y; ly <= ground_max_y; ly += 10.0){
		glVertex3d(-ground_max_x, ly, -1.1);
		glVertex3d(ground_max_x, ly, -1.1);
	}
	for (double lx = -ground_max_x; lx <= ground_max_x; lx += 10.0){
		glVertex3d(lx, ground_max_y, -1.1);
		glVertex3d(lx, -ground_max_y, -1.1);
	}
	glEnd();
}

//----------------------------------------------------
// �A�C�h�����ɌĂяo�����֐�
//----------------------------------------------------
//void Idle(){
//	glutPostRedisplay(); //glutDisplayFunc()���P����s����
//}
//----------------------------------------------------
// �^�C�}�[�ŌĂяo�����֐�
//----------------------------------------------------
void timer(int value){
	glutPostRedisplay(); //glutDisplayFunc()���P����s����
	glutTimerFunc(30, timer, value);
}
//----------------------------------------------------
// �L�[�{�[�h���͎��ɌĂяo�����֐�
//----------------------------------------------------
void Keyboard(unsigned char key, int x, int y){
	switch (key)
	{
	case 'q':
		exit(0);
		break;
	default:
		break;
	}
}
//----------------------------------------------------
// �����ʂ̕������ƍs��̌v�Z
//----------------------------------------------------
void findPlane(
	GLfloat plane[4],  // �쐬���镽�ʕ������̌W��
	GLfloat v0[3],    // ���_�P
	GLfloat v1[3],    // ���_�Q
	GLfloat v2[3])    // ���_�R
{
	GLfloat vec0[3], vec1[3];

	// Need 2 vectors to find cross product.
	vec0[0] = v1[0] - v0[0];
	vec0[1] = v1[1] - v0[1];
	vec0[2] = v1[2] - v0[2];

	vec1[0] = v2[0] - v0[0];
	vec1[1] = v2[1] - v0[1];
	vec1[2] = v2[2] - v0[2];

	// find cross product to get A, B, and C of plane equation
	plane[0] = vec0[1] * vec1[2] - vec0[2] * vec1[1];
	plane[1] = -(vec0[0] * vec1[2] - vec0[2] * vec1[0]);
	plane[2] = vec0[0] * vec1[1] - vec0[1] * vec1[0];

	plane[3] = -(plane[0] * v0[0] + plane[1] * v0[1] + plane[2] * v0[2]);
}
void shadowMatrix(
	GLfloat *m,      // �쐬����s��̃|�C���^
	GLfloat plane[4],  // �ˉe����\�ʂ̕��ʕ������̌W��
	GLfloat light[4])  // �����̓������W�l
{
	GLfloat dot;

	// Find dot product between light position vector and ground plane normal.
	dot = plane[0] * light[0] +
		plane[1] * light[1] +
		plane[2] * light[2] +
		plane[3] * light[3];

	m[0] = dot - light[0] * plane[0];
	m[4] = 0.f - light[0] * plane[1];
	m[8] = 0.f - light[0] * plane[2];
	m[12] = 0.f - light[0] * plane[3];

	m[1] = 0.f - light[1] * plane[0];
	m[5] = dot - light[1] * plane[1];
	m[9] = 0.f - light[1] * plane[2];
	m[13] = 0.f - light[1] * plane[3];

	m[2] = 0.f - light[2] * plane[0];
	m[6] = 0.f - light[2] * plane[1];
	m[10] = dot - light[2] * plane[2];
	m[14] = 0.f - light[2] * plane[3];

	m[3] = 0.f - light[3] * plane[0];
	m[7] = 0.f - light[3] * plane[1];
	m[11] = 0.f - light[3] * plane[2];
	m[15] = dot - light[3] * plane[3];
}

//----------------------------------------------------
// ���̕`��Ɖe�̕`��
//----------------------------------------------------
void DrawFloor(bool bTexture){
	if (bTexture){
		// ���Ƀe�N�X�`�����g�����̓R�R�Őݒ肷��
		//  glBindTexture( GL_TEXTURE_2D, );

		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		//    glTexCoord2f( , );
		glVertex3fv(floor_v.v0);
		//    glTexCoord2f( , );
		glVertex3fv(floor_v.v1);
		//    glTexCoord2f( , );
		glVertex3fv(floor_v.v2);
		//    glTexCoord2f( , );
		glVertex3fv(floor_v.v3);
		glEnd();
		glEnable(GL_LIGHTING);
	}
	else{
		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		glVertex3fv(floor_v.v0);
		glVertex3fv(floor_v.v1);
		glVertex3fv(floor_v.v2);
		glVertex3fv(floor_v.v3);
		glEnd();
		glEnable(GL_LIGHTING);
	}
}
void DrawShadow(void){
	/////////////////////////////////////////////
	//���̃X�e���V����t����
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 1, ~0);
	//���ꂩ��`�悷����̂̃X�e���V���l�ɂ��ׂĂP�^�O������
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	glColor4f(0.7f, 0.4f, 0.0f, 1.0f);
	DrawFloor(true);//���̕`��

	/////////////////////////////////////////////
	//�J���[�E�f�v�X�o�b�t�@�}�X�N���Z�b�g����
	//����ňȉ��̓��e�̃s�N�Z���̐F�̒l�́A�������܂�Ȃ��B
	glColorMask(0, 0, 0, 0);
	glDepthMask(0);
	/////////////////////////////////////////////
	//���ɃI�u�W�F�N�g�̉e�̃X�e���V����t����
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_EQUAL, 1, ~0);
	//���ꂩ��`�悷����̂̃X�e���V���l�ɂ��ׂĂP�^�O������
	glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
	glDisable(GL_DEPTH_TEST);
	glPushMatrix();
	glMultMatrixf(pM);
	DrawStructure(true);
	glPopMatrix();
	glEnable(GL_DEPTH_TEST);

	/////////////////////////////////////////////
	//�r�b�g�}�X�N������
	glColorMask(1, 1, 1, 1);
	glDepthMask(1);

	/////////////////////////////////////////////
	//�e������
	glStencilFunc(GL_EQUAL, 2, ~0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.1f, 0.1f, 0.1f, 0.5f);
	glDisable(GL_DEPTH_TEST);
	DrawFloor(false);//���̕`��
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_STENCIL_TEST);
}
