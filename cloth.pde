//PDEs and Integration
//CSCI 5611 Swinging Rope [Exercise]
//Stephen J. Guy <sjguy@umn.edu>

//NOTE: The simulation starts paused, press "space" to unpause

//Create Window
String windowTitle = "Swinging Rope";
void setup() {
  size(800, 1000, P3D);
  surface.setTitle(windowTitle);
  camera = new Camera();
  initScene();
}

Camera camera;
//Simulation Parameters
float floor = 1000;
Vec2 gravity = new Vec2(0,200,0);
float radius = 1;
Vec2 stringTop[] = new Vec2[maxNodes];

float mu = .75;
float restLen = 10;
float mass = 1.0; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 2000; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 100; //TRY-IT: How big can you make kv?

//Initial positions and velocities of masses
static int maxNodes = 100;
int numRopes = 40;
int numNodes = 25;
float sphereRad = 30;
float sphereCon = 30;
float a=195.0,b=160.0,c=-35.0;
float A=200,B=200,C=580;

Vec2 pos[][] = new Vec2[numRopes][numNodes];
Vec2 vel[][] = new Vec2[numRopes][numNodes];
Vec2 acc[][] = new Vec2[numRopes][numNodes];

void initScene(){                      //Initialization
  for (int j=0; j<numRopes; j++){
    stringTop[j] = new Vec2(50+15*j,50,0);
    
    for (int i = 0; i < numNodes; i++){
      pos[j][i] = new Vec2(0,0,0);
      pos[j][i].x = stringTop[j].x;
      pos[j][i].y = stringTop[j].y; 
      pos[j][i].z = stringTop[j].z + 10*i;
      vel[j][i] = new Vec2(0,0,0);
      acc[j][i] = new Vec2(0,0,0);
    }
  }
}

void update(float dt){

  //Reset accelerations each timestep (momentum only applies to velocity)
  Vec2 vell[][] = new Vec2[numRopes][numNodes];
  for (int j = 0; j < numRopes; j++){
    for (int i = 0; i < numNodes; i++){
      acc[j][i] = new Vec2(0,0,0);
      acc[j][i].add(gravity);
      vell[j][i] = vel[j][i];
      
      Vec2 test = new Vec2(a,b,c);
      float dist = test.distanceTo(pos[j][i]);
      if(dist < sphereCon + 0.09){          //Check for collision with special sphere
        Vec2 norm = (test.minus(pos[j][i])).times(-1.0);
        norm.normalize();
        Vec2 bounce = norm.times(dot(vel[j][i],norm));
        vel[j][i].subtract(bounce.times(1.5));
        pos[j][i].add(norm.times(.1+sphereCon-dist));
      }
      
      test = new Vec2(A,B,C);
      dist = test.distanceTo(pos[j][i]);
      if(dist < sphereRad + 0.2){          //Check for collision with random sphere
        Vec2 norm = (test.minus(pos[j][i])).times(-1.0);
        norm.normalize();
        Vec2 bounce = norm.times(dot(vel[j][i],norm));
        vel[j][i].subtract(bounce.times(1.5));
        pos[j][i].add(norm.times(.1+sphereRad-dist));
      }
    }
  }
 
  //Compute (damped) Hooke's law for each spring

  for (int j = 0; j < numRopes-1; j++){
    for (int i = 0; i < numNodes; i++){
      Vec2 e = pos[j+1][i].minus(pos[j][i]);
      float l = sqrt(dot(e,e));
      e.normalize();
      float v1 = dot(e,vel[j][i]);
      float v2 = dot(e,vel[j+1][i]);
      float f = -k*(10-l) - kv*(v1-v2);
      vell[j][i].add(e.times(f*dt));
      vell[j+1][i].subtract(e.times(f*dt));
    }
  }
  for (int j = 0; j < numRopes; j++){
    for (int i = 0; i < numNodes-1; i++){
      Vec2 e = pos[j][i+1].minus(pos[j][i]);
      float l = sqrt(dot(e,e));
      e.normalize();
      float v1 = dot(e,vel[j][i]);
      float v2 = dot(e,vel[j][i+1]);
      float f = -k*(10-l) - kv*(v1-v2);
      vell[j][i].add(e.times(f*dt));
      vell[j][i+1].subtract(e.times(f*dt));
    }
  }
  vel = vell;
    
  for (int j = 0; j < numRopes; j++){
    vel[j][0] = new Vec2(0,0,0);
    //Eulerian integration
    for (int i = 1; i < numNodes; i++){
      vel[j][i].add(acc[j][i].times(dt));
      pos[j][i].add(vel[j][i].times(dt));
    }
  }
}


boolean paused = true;
void draw() {
  background(255,255,255);
  if (!paused){ 
    for (int i = 0; i < 40; i++){
      update(1/(40*frameRate));
    }    
    C-=1;
  }
  camera.Update(1/frameRate);
  fill(0,0,0);
 
  //Draw rope nodes
  for (int j = 0; j < numRopes; j++){
    pushMatrix();
    line(pos[j][0].x,pos[j][0].y,pos[j][0].z,pos[j][1].x,pos[j][1].y,pos[j][1].z);
    translate(pos[j][0].x,pos[j][0].y,pos[j][0].z);
    popMatrix();
    for (int i = 0; i < numNodes-1; i++){
      pushMatrix();
      line(pos[j][i].x,pos[j][i].y,pos[j][i].z,pos[j][i+1].x,pos[j][i+1].y,pos[j][i+1].z);
      translate(pos[j][i+1].x,pos[j][i+1].y,pos[j][i+1].z);
      popMatrix();
    }
  }
  for (int j = 0; j < numRopes-1; j++){
    for (int i = 0; i < numNodes; i++){
      pushMatrix();
      line(pos[j][i].x,pos[j][i].y,pos[j][i].z,pos[j+1][i].x,pos[j+1][i].y,pos[j+1][i].z);
      translate(pos[j+1][i].x,pos[j+1][i].y,pos[j+1][i].z);
      popMatrix();
    }
  }

  //Draw circle object
  fill(20,60,250);
  pushMatrix();
  translate(A,B,C);
  sphere(sphereRad);
  popMatrix();
  pushMatrix();
  translate(a,b,c);
  sphere(sphereCon);
  popMatrix();
  if(C <= -220){
    C = 300;
    A = random(401)+200;
    B = random(151)+125;
  }
  
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

boolean f=true;
float speed=5;
void keyPressed(){
  if (key == 'f'){
    f=!f;
  }else if (key == 'r'){
    initScene();
  }else if (key == ' ') paused = !paused;
  if(f){
    if ( keyCode == LEFT  ){
       a-=speed;
    }
    if ( keyCode == RIGHT ){
       a+=speed;
    }
    if ( keyCode == UP    ){
       b-=speed;
    }
    if ( keyCode == DOWN  ){
       b+=speed;
    }
    if ( key == 'q' ){
       c-=speed;
    }
    if ( key == 'e'  ){
       c+=speed;
    }
  }else camera.HandleKeyPressed();
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

///////////////////
// Vec2D Library
///////////////////
public class Camera
{
  Camera()
  {
    position      = new PVector(-312.09927, 56.06335, 249.58913);//new PVector( 250, 250, 800 );//new PVector( 1322.3567, 196.43929, 485.43295 ); // initial position
    theta         = -1.0874559;//1.1669071; // rotation around Y axis. Starts with forward direction as ( 0, 0, -1 )
    phi           = -0.258376;//-0.23024613; // rotation around X axis. Starts with up direction as ( 0, 1, 0 )
    moveSpeed     = 50;
    turnSpeed     = 1.57; // radians/sec
    boostSpeed    = 10;  // extra speed boost for when you press shift
    
    // dont need to change these
    shiftPressed = false;
    negativeMovement = new PVector( 0, 0, 0 );
    positiveMovement = new PVector( 0, 0, 0 );
    negativeTurn     = new PVector( 0, 0 ); // .x for theta, .y for phi
    positiveTurn     = new PVector( 0, 0 );
    fovy             = PI / 4;
    aspectRatio      = width / (float) height;
    nearPlane        = 0.1;
    farPlane         = 10000;
  }
  
  void Update(float dt)
  {
    theta += turnSpeed * ( negativeTurn.x + positiveTurn.x)*dt;
    
    // cap the rotation about the X axis to be less than 90 degrees to avoid gimble lock
    float maxAngleInRadians = 85 * PI / 180;
    phi = min( maxAngleInRadians, max( -maxAngleInRadians, phi + turnSpeed * ( negativeTurn.y + positiveTurn.y ) * dt ) );
    
    // re-orienting the angles to match the wikipedia formulas: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    // except that their theta and phi are named opposite
    float t = theta + PI / 2;
    float p = phi + PI / 2;
    PVector forwardDir = new PVector( sin( p ) * cos( t ),   cos( p ),   -sin( p ) * sin ( t ) );
    PVector upDir      = new PVector( sin( phi ) * cos( t ), cos( phi ), -sin( t ) * sin( phi ) );
    PVector rightDir   = new PVector( cos( theta ), 0, -sin( theta ) );
    PVector velocity   = new PVector( negativeMovement.x + positiveMovement.x, negativeMovement.y + positiveMovement.y, negativeMovement.z + positiveMovement.z );
    position.add( PVector.mult( forwardDir, moveSpeed * velocity.z * dt ) );
    position.add( PVector.mult( upDir,      moveSpeed * velocity.y * dt ) );
    position.add( PVector.mult( rightDir,   moveSpeed * velocity.x * dt ) );
    
    aspectRatio = width / (float) height;
    perspective( fovy, aspectRatio, nearPlane, farPlane );
    camera( position.x, position.y, position.z,
            position.x + forwardDir.x, position.y + forwardDir.y, position.z + forwardDir.z,
            upDir.x, upDir.y, upDir.z );
  }
  
  // only need to change if you want difrent keys for the controls
  void HandleKeyPressed()
  {
    if ( key == 'w' || key == 'W' ) positiveMovement.z = 1;
    if ( key == 's' || key == 'S' ) negativeMovement.z = -1;
    if ( key == 'a' || key == 'A' ) negativeMovement.x = -1;
    if ( key == 'd' || key == 'D' ) positiveMovement.x = 1;
    if ( key == 'q' || key == 'Q' ) positiveMovement.y = 1;
    if ( key == 'e' || key == 'E' ) negativeMovement.y = -1;
    
    if ( key == 'r' || key == 'R' ){
      Camera defaults = new Camera();
      position = defaults.position;
      theta = defaults.theta;
      phi = defaults.phi;
    }
    
    if ( keyCode == LEFT )  negativeTurn.x = 1;
    if ( keyCode == RIGHT ) positiveTurn.x = -0.5;
    if ( keyCode == UP )    positiveTurn.y = 0.5;
    if ( keyCode == DOWN )  negativeTurn.y = -1;
    
    if ( keyCode == SHIFT ) shiftPressed = true; 
    if (shiftPressed){
      positiveMovement.mult(boostSpeed);
      negativeMovement.mult(boostSpeed);
    }
    
  }
  
  // only need to change if you want difrent keys for the controls
  void HandleKeyReleased()
  {
    if ( key == 'w' || key == 'W' ) positiveMovement.z = 0;
    if ( key == 'q' || key == 'Q' ) positiveMovement.y = 0;
    if ( key == 'd' || key == 'D' ) positiveMovement.x = 0;
    if ( key == 'a' || key == 'A' ) negativeMovement.x = 0;
    if ( key == 's' || key == 'S' ) negativeMovement.z = 0;
    if ( key == 'e' || key == 'E' ) negativeMovement.y = 0;
    
    if ( keyCode == LEFT  ) negativeTurn.x = 0;
    if ( keyCode == RIGHT ) positiveTurn.x = 0;
    if ( keyCode == UP    ) positiveTurn.y = 0;
    if ( keyCode == DOWN  ) negativeTurn.y = 0;
    
    if ( keyCode == SHIFT ){
      shiftPressed = false;
      positiveMovement.mult(1.0/boostSpeed);
      negativeMovement.mult(1.0/boostSpeed);
    }
  }
  
  // only necessary to change if you want different start position, orientation, or speeds
  PVector position;
  float theta;
  float phi;
  float moveSpeed;
  float turnSpeed;
  float boostSpeed;
  
  // probably don't need / want to change any of the below variables
  float fovy;
  float aspectRatio;
  float nearPlane;
  float farPlane;  
  PVector negativeMovement;
  PVector positiveMovement;
  PVector negativeTurn;
  PVector positiveTurn;
  boolean shiftPressed;
};



public class Vec2 {
  public float x, y, z;
  
  public Vec2(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public float lengthSqr(){
    return x*x+y*y+z*z;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y, z+rhs.z);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y, z-rhs.z);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs, z*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y + z*z);
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y + z*z);
    return new Vec2(x/magnitude, y/magnitude, z/magnitude);
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
      z *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y + z*z);
    x *= newL/magnitude;
    y *= newL/magnitude;
    z *= newL/magnitude;
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float dz = rhs.z - z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }
  
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}
