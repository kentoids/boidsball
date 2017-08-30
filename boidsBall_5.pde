/**
 * Flocking
 * by Daniel Shiffman.
 * edited by Kento Morita
 */

Flock flock;
import peasy.*;

PeasyCam cam;
int mode = 0;

void setup() {
  size(900, 900, P3D);
  sphereDetail(10);
  cam = new PeasyCam(this, 200);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(500);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < 150; i++) {
    flock.addBoid(new Boid(width/2,height/2));
  }
}

void draw() {
  background(50);
  flock.run(mode);
}

// Add a new boid into the System
// void mousePressed() {
//   flock.addBoid(new Boid(mouseX,mouseY));
// }

void keyPressed() {
  if (key == '0') {
    mode = 0;
  }
  if (key == '1') {
    mode = 1;
  }
  if (key == '2') {
    mode = 2;
  }
  if (key == '3') {
    mode = 3;
  }
  if (key == '4') {
    mode = 4;
  }
  if (key == '5') {
    mode = 5;
  }
  if (key == '6') {
    mode = 6;
  }
  if (key == '9') {
    flock.colorChange();
  }
  if (key == ' ') {
    saveFrame("img/#####.png");
  }
}
