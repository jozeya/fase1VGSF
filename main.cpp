#include <iostream>
#include "Vehicle.h"

# include <GL/gl.h>
# include <GL/glu.h>
# include <GL/glut.h>

using namespace std;

Vehicle *vehicle;
Vehicle *vehicle2;
Vehicle *vehicle3;
Vehicle *vehicle4;
Vehicle *vehicle5;

Vehicle *uBullet;

bool shoot = false;
Vector2D* setShoot;


Vector2D *crosshair;
Vector2D cPointerMouse;
double i = 0;
int cx = 640;
int cy = 480;

vector<Wall2D> m_Walls;

//enum Deceleration{slow = 3, normal = 2, fast = 1};

GLvoid callback_mouse(int button, int state, int x, int y);
GLvoid callback_motion(int, int);
GLvoid callback_special(int, int, int);
GLvoid ShowVectorsVehicle(const Vehicle*);
GLvoid DrawRadiusWander(Vehicle*);
GLvoid CreateWalls();
GLvoid DrawWalls();

float x_move_mouse, tmp_x_move;
float y_move_mouse, tmp_y_move;

double r;

void Circle(float px,float py,float r){
	glBegin(GL_LINE_LOOP);
	//glColor3d(255,0,0);

	// 0 < t < 2 PI
	float t = 0.0, x = 0.0, y = 0.0;
	for (int i = 0 ; i < 50 ; i++){
		t = float(i) * 2.0 * Pi;
		x = r * cosf(t/50);
		y = r * sinf(t/50);
		glVertex2d(px + x, py + y);
	}

	glEnd();
}

void RenderCrosshair(Vector2D ch){

	//glColor3f(1,0,1);
	glPushMatrix();
	Circle((float)ch.x, (float)ch.y, 10);
	glBegin(GL_LINES);
		glVertex2d(ch.x - 18, ch.y); glVertex2d(ch.x + 18, ch.y);
  		glVertex2d(ch.x, ch.y - 18); glVertex2d(ch.x, ch.y + 18);
  	glEnd();
  	glPopMatrix();
}



void PrintVec(const std::vector<Vector2D> &points){
	for (unsigned int p = 0; p < points.size(); ++p){
    	Vector2D tmp = points[p];

    	cout << tmp.x << ", " << tmp.y << endl;
    }	
    cout << endl;
}


void ClosedShape(const std::vector<Vector2D> &points){
	glPushMatrix();
	glBegin(GL_POLYGON);
		glVertex2d(points[0].x, points[0].y); 

	    for (unsigned int p=1; p<points.size(); ++p){
    		glVertex2d(points[p].x, points[p].y);
    	}
    	glVertex2d(points[0].x, points[0].y); 

    glEnd();
    glPopMatrix();
}

void reshape_cb (int w, int h) {
	if (w==0||h==0) return;
	glViewport(0,0,w,h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluOrtho2D(0,w,0,h);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
}

void rotateVehicle(int x, int y){
	Vector2D vRotate(x,480 - y);
	vehicle -> rotateHeadingToFacePosition(vRotate);
}

void moveObject(){

}


double timer_elapsed = 0.04;
void display_cb() {
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1,1,0); glLineWidth(3);

	//vehicle2 -> Flee(vehicle3 -> getPos());
	//vehicle -> Seek(*crosshair);
	//vehicle2 -> Update(timer_elapsed);
	//vehicle -> Update(timer_elapsed);

	//vehicle3 -> Pursuit(vehicle2);
	//vehicle3 -> Update(timer_elapsed);

	//vehicle4 -> WanderOn();
	//vehicle4 -> Update(timer_elapsed);
	//DrawWalls();

	vehicle2 -> CalculatePrioritized(m_Walls, vehicle3 -> getPos());
	vehicle2 -> Update(timer_elapsed);

	vehicle4 -> CalculatePrioritized(m_Walls, *crosshair);
	vehicle4 -> Update(timer_elapsed);

	vehicle3 -> CalculatePrioritized(m_Walls, *crosshair);
	vehicle3 -> Update(timer_elapsed);

	ClosedShape(vehicle2 -> getVehicleBuffTrans());
	ClosedShape(vehicle -> getVehicleBuffTrans());
	ClosedShape(vehicle3 -> getVehicleBuffTrans());
	ClosedShape(vehicle4 -> getVehicleBuffTrans());
	ClosedShape(uBullet -> getVehicleBuffTrans());

	//ShowVectorsVehicle(vehicle3);
	//ShowVectorsVehicle(vehicle2);
	//ShowVectorsVehicle(vehicle);

	DrawRadiusWander(vehicle2);
	DrawRadiusWander(vehicle3);
	DrawRadiusWander(vehicle4);
	//DrawRadiusWander(vehicle2);
	

	RenderCrosshair(*crosshair);


	if (shoot){
		uBullet -> CalculatePrioritized(m_Walls, *setShoot);
		uBullet -> Update(timer_elapsed);

		double PanicDistanceSq = 100.0 ;

		if (Vec2DDistanceSq(uBullet -> getPos(), *setShoot) < PanicDistanceSq){
			uBullet -> setPos(vehicle -> getPos());
			shoot = false;
		}
	}

	//timer_elapsed +=0.0001;

	glutSwapBuffers();
}

GLvoid window_idle(){
	glutPostRedisplay();
}

void initialize() {
	glutInitDisplayMode (GLUT_RGBA|GLUT_DOUBLE);
	glutInitWindowSize (640,480);
	glutInitWindowPosition (100,100);
	glutCreateWindow ("Ventana OpenGL");
	glutDisplayFunc (display_cb);
	glutReshapeFunc (reshape_cb);
	glutIdleFunc(&window_idle);
	glutSpecialFunc(&callback_special);
	glutMouseFunc(&callback_mouse);
	glutPassiveMotionFunc(&callback_motion);
	glClearColor(0.f,0.f,0.f,0.f);
}

int main(int argc, char **argv){
	int cx = 640;
	int cy = 480;

	crosshair = new Vector2D(cx/3.0, cy/3.0);

	Vector2D SpawnPos = Vector2D(cx/2.0+RandomClamped()*cx/2.0, cy/2.0+RandomClamped()*cy/2.0);
	Vector2D SpawnPos2 = Vector2D(cx/2.0, cy/2.0);
	Vector2D SpawnPos3 = Vector2D(cx/3.0, cy/3.0);
	Vector2D SpawnPos4 = Vector2D(cx/4.0, cy/4.0);

	vehicle = new Vehicle(SpawnPos2, RandFloat()*TwoPi, Vector2D(0.0, 0.0), 1.0, 200.0 * 2.0, 150.0, Pi, 15.0);
	vehicle2 = new Vehicle(SpawnPos, RandFloat()*TwoPi, Vector2D(0.0, 0.0), 1.0, 200.0 * 2.0, 100.0, Pi, 15.0);
	vehicle3 = new Vehicle(SpawnPos3, RandFloat()*TwoPi, Vector2D(0.0, 0.0), 6.0, 200.0 * 2.0, 100.0, Pi, 15.0);
	vehicle4 = new Vehicle(SpawnPos4, RandFloat()*TwoPi, Vector2D(0.0, 0.0), 1.0, 200.0 * 2.0, 100.0, Pi, 15.0);

	uBullet = new Vehicle(SpawnPos2, RandFloat()*TwoPi, Vector2D(0.0, 0.0), 1.0, 200.0 * 2.0, 80.0, Pi, 5.0);


	CreateWalls();

	uBullet -> SeekOn();
	vehicle -> SeekOn();
	//vehicle2 -> EvadeOn(vehicle3);
	//vehicle2 -> FleeOn();
	vehicle2 -> WanderOn();
	vehicle2 -> EvadeOn(uBullet);
	//vehicle2 -> WallAvoidanceOn();

	vehicle3 -> WanderOn();
	vehicle3 -> EvadeOn(uBullet);
	//vehicle3 -> WallAvoidanceOn();	
	//vehicle3 -> PursuitOn(vehicle2);
	//vehicle3 -> WallAvoidanceOn();

	vehicle4 -> WanderOn();
	vehicle4 -> EvadeOn(uBullet);
	//vehicle4 -> WallAvoidanceOn();

	glutInit (&argc, argv);
	initialize();
	glutMainLoop();

	return 0;
}

GLvoid ShowVectorsVehicle(const Vehicle* vehicle){
	int x = 40;
	glPushMatrix();
		glColor3f(0,0,1);
		glBegin(GL_LINES);
			glVertex2d(vehicle -> getPos().x, vehicle -> getPos().y); glVertex2d(vehicle -> getPos().x + vehicle -> getHeading().x*x, 
																				 vehicle -> getPos().y + vehicle -> getHeading().y*x);
  		glEnd();
  	glPopMatrix();

  	glPushMatrix();
		glColor3f(1,0,0);
		glBegin(GL_LINES);
			glVertex2d(vehicle -> getPos().x, vehicle -> getPos().y); glVertex2d(vehicle -> getPos().x + vehicle -> getSide().x*x, 
																				 vehicle -> getPos().y + vehicle -> getSide().y*x);
  		glEnd();
  	glPopMatrix();


  	glutPostRedisplay();
}


GLvoid callback_mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON){

		shoot = 1;
		setShoot = new Vector2D(x, 480 - y);
		uBullet -> setPos(vehicle -> getPos());
	}
}

GLvoid callback_special(int key, int x, int y)
{

	switch (key){
		case GLUT_KEY_UP:

			vehicle -> CalculatePrioritized(m_Walls, cPointerMouse);
			vehicle -> Update(timer_elapsed);
			if (!shoot)
				uBullet -> setPos(vehicle -> getPos());
			glutPostRedisplay();			// et on demande le réaffichage.
			break;

		case GLUT_KEY_DOWN:
			r = vehicle4 -> getWanderRadius();
			r += 1;
			vehicle4 -> setWanderRadius(r);
			glutPostRedisplay();			// et on demande le réaffichage.
			break;

		case GLUT_KEY_LEFT:

			glutPostRedisplay();			// et on demande le réaffichage.
			break;

		case GLUT_KEY_RIGHT:				
			
			glutPostRedisplay();			// et on demande le réaffichage.
			break;
	}
}

GLvoid callback_motion(int x, int y)
{
	rotateVehicle(x,y);
	cPointerMouse = Vector2D(x,480 - y);
	crosshair = new Vector2D(cPointerMouse);
	glutPostRedisplay();						
}


GLvoid DrawRadiusWander(Vehicle* m_pVehicle){
	Vector2D m_vTCC = PointToWorldSpace(Vector2D(m_pVehicle -> getWanderDistance() * m_pVehicle->getRadius(), 0),
                                         m_pVehicle->getHeading(),
                                         m_pVehicle->getSide(),
                                         m_pVehicle->getPos());
	glPushMatrix();
		glColor3f(1,0,0);
		Circle(m_vTCC.x, m_vTCC.y, m_pVehicle -> getWanderRadius() * m_pVehicle->getRadius()); 
	glPopMatrix();


	m_vTCC = PointToWorldSpace(( m_pVehicle -> getWanderTarget() + Vector2D(m_pVehicle -> getWanderDistance(),0)) * m_pVehicle->getRadius(),
                                  m_pVehicle->getHeading(),
                                  m_pVehicle->getSide(),
                                  m_pVehicle->getPos());

	glPushMatrix();
		glColor3f(0,1,0);
		Circle(m_vTCC.x, m_vTCC.y, 3);
	glPopMatrix();

	m_pVehicle -> Seek(m_vTCC);
	//m_pVehicle -> setSteeringForce( m_pVehicle -> WallAvoidance(m_Walls) );
}

GLvoid CreateWalls(){
  //create the walls  
  double bordersize = 20.0;
  double CornerSize = 0.2;
  double vDist = cy - 2 * bordersize;
  double hDist = cx -2 * bordersize;
  
  const int NumWallVerts = 8;

  Vector2D walls[NumWallVerts] = {Vector2D(hDist * CornerSize + bordersize, bordersize),
                                   Vector2D(cx-bordersize-hDist*CornerSize, bordersize),
                                   Vector2D(cx-bordersize, bordersize+vDist*CornerSize),
                                   Vector2D(cx-bordersize, cy - bordersize-vDist*CornerSize),
                                         
                                   Vector2D(cx-bordersize-hDist*CornerSize, cy - bordersize),
                                   Vector2D(hDist*CornerSize+bordersize, cy -bordersize),
                                   Vector2D(bordersize, cy - bordersize-vDist*CornerSize),
                                   Vector2D(bordersize, bordersize+vDist*CornerSize)};
  
  for (int w=0; w < NumWallVerts-1; ++w){
    m_Walls.push_back(Wall2D(walls[w], walls[w+1]));
  }

  m_Walls.push_back(Wall2D(walls[NumWallVerts-1], walls[0]));
}

GLvoid DrawWalls(){
	for (unsigned int w=0; w<m_Walls.size(); ++w){
    	m_Walls[w].Render(true);  //true flag shows normals
  	}

}