// The Boid class

class Boid {

  PVector location;
  PVector velocity;
  PVector acceleration;
  PVector polarloc;
  float r;
  float radius = 100;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed

  int mode = 0; // visualization modes
  boolean colorOn = false;
  int colorMode = 0;
  int colorNum = 5;
  color mycolor;
  ArrayList<PVector> prevloc = new ArrayList<PVector>();
  /*
    0 : normal
    1 : line to near boid (separate)
    2 : line to near boid (recognized)
    3 : line to all other boids
    4 : draw track
    5 : draw track (thicker line)
    6 : draw arc
    9 : change colors
  */

    Boid(float x, float y) {
    acceleration = new PVector(0, 0);

    // This is a new PVector method not yet implemented in JS
    // velocity = PVector.random2D();

    // Leaving the code temporarily this way so that this example runs in JS
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));

    location = new PVector(x, y);
    polarloc = convertToPolar(location);
    r = 2.0;
    maxspeed = 2;
    maxforce = 0.03;
  }

  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
  }

  void run(ArrayList<Boid> boids, int _mode) {
    mode = _mode;
    flock(boids);
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }

  // Method to update location
  void update() {
    prevloc.add(polarloc);
    if (prevloc.size() > 200) {
      prevloc.remove(0);
    }
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    PVector newvel = new PVector(velocity.x/50, velocity.y/50);
    location.add(newvel);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
    // update polar location
    polarloc = convertToPolar(location);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);  // A vector pointing from the location to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);

    // Above two lines of code below could be condensed with new PVector setMag() method
    // Not using this method until Processing.js catches up
    // desired.setMag(maxspeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    // PVector velnorm = velocity.normalize();
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up

    // PVector moveto = convertToPolar(location);
    sphereDetail(8);
    switch(mode) {
      case 0:
        pushMatrix();
        fill(255);
        noStroke();
        translate(polarloc.x, polarloc.y, polarloc.z);
        sphere(r);
        popMatrix();
        break;
      case 4:
        strokeWeight(3);
        noFill();
        beginShape();
        for(int i=0; i<prevloc.size()/2; i++) {
          PVector ploc = prevloc.get(i);
          float inorm = map(i, 0, prevloc.size()/2, 0, 1);
          if (colorMode == 0) {
            // fill(200, 255 * inorm);
            stroke(200, 255 * inorm);
          } else {
            // fill(mycolor, 255 * inorm);
            stroke(mycolor, 255 * inorm);
          }
          vertex(ploc.x, ploc.y, ploc.z);
        }
        endShape();
        break;
      case 5:
      strokeWeight(6);
      noFill();
      beginShape();
      for(int i=0; i<prevloc.size(); i++) {
        PVector ploc = prevloc.get(i);
        float inorm = map(i, 0, prevloc.size(), 0, 1);
        if (colorMode == 0) {
          // fill(200, 255 * inorm);
          stroke(200, 255 * inorm);
        } else {
          // fill(mycolor, 255 * inorm);
          stroke(mycolor, 255 * inorm);
        }
        vertex(ploc.x, ploc.y, ploc.z);
      }
      endShape();
      break;
      case 6:
        strokeWeight(3);
        beginShape();
        for(int i=0; i<prevloc.size()/2; i++) {
          PVector ploc = prevloc.get(i);
          float inorm = map(i, 0, prevloc.size()/2, 0, 1);
          if (colorMode == 0) {
            fill(200, 255 * inorm);
            stroke(200, 255 * inorm);
          } else {
            fill(mycolor, 255 * inorm);
            stroke(mycolor, 255 * inorm);
          }
          vertex(ploc.x, ploc.y, ploc.z);
        }
        endShape();
        break;
      }
  }

  PVector convertToPolar(PVector _location) {
    // convert from othogonal coordinate system to polar
    float x = radius * sin(_location.x) * cos(_location.y);
    float y = radius * sin(_location.x) * sin(_location.y);
    float z = radius * cos(_location.x);
    return new PVector(x,y,z);
  }

  // Wraparound
  void borders() {
    // 2*PI used to be WIDTH and HEIGHT
    if (location.x < 0) location.x += 2*PI;
    if (location.y < 0) location.y += 2*PI;
    if (location.x > 2*PI) location.x -= 2*PI;
    if (location.y > 2*PI) location.y -= 2*PI;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.0f;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(polarloc, other.polarloc);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many

        if (mode == 1) {
          // draw inside separate to save distance calculation
          strokeWeight(1);
          beginShape(LINES);
          if (colorMode == 0) {
            stroke(200, 150);
          } else {
            stroke(mycolor);
          }
          vertex(polarloc.x, polarloc.y, polarloc.z);
          vertex(other.polarloc.x, other.polarloc.y, other.polarloc.z);
          endShape();
        }
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // steer.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(polarloc, other.polarloc);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;

        // draw all recognized boids
        if (mode == 2) {
          // draw inside separate to save distance calculation
          strokeWeight(1);
          beginShape(LINES);
          // stroke(0);
          if (colorMode == 0) {
            stroke(200, 150);
          } else {
            stroke(mycolor);
          }
          vertex(polarloc.x, polarloc.y, polarloc.z);
          vertex(other.polarloc.x, other.polarloc.y, other.polarloc.z);
          endShape();
        }
      }
      // draw line to all boids
      if (mode == 3) {
        // draw inside separate to save distance calculation
        strokeWeight(1);
        beginShape(LINES);
        if (colorMode == 0) {
          stroke(200, 150);
        } else {
          stroke(mycolor);
        }
        vertex(polarloc.x, polarloc.y, polarloc.z);
        vertex(other.polarloc.x, other.polarloc.y, other.polarloc.z);
        endShape();
      }
    }
    if (count > 0) {
      sum.div((float)count);
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // sum.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer;
    }
    else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(polarloc, other.polarloc);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the location
    }
    else {
      return new PVector(0, 0);
    }
  }
  void changeColorMode() {
    colorMode++;
    colorMode = colorMode % colorNum;
    colorSetter(colorMode);
  }
  void colorSetter(int _setTo) {
    // set color
    if (_setTo == 1) {
      color c1 = lerpColor(color(106, 152, 199, 200), color(200, 150), random(1));
      mycolor = c1;
    } else if (_setTo == 2) {
      int rand = floor(random(5));
      if (rand == 1) {
        mycolor = #DEE5E5;
      } else if (rand == 2) {
        mycolor = #9DC5BB;
      } else if (rand == 3) {
        mycolor = #17B890;
      } else if (rand == 4) {
        mycolor = #5E807F;
      } else if (rand == 5) {
        mycolor = #082D0F;
      }
      mycolor = color(mycolor, 150);
    } else if (_setTo == 3) {
      int rand = floor(random(5));
      if (rand == 1) {
        mycolor = #203309;
      } else if (rand == 2) {
        mycolor = #FDC8F1;
      } else if (rand == 3) {
        mycolor = #E16CBC;
      } else if (rand == 4) {
        mycolor = #7B25C7;
      } else if (rand == 5) {
        mycolor = #050B85;
      }
      mycolor = color(mycolor, 150);
    } else if (_setTo == 4) {
      int rand = floor(random(5));
      if (rand == 1) {
        mycolor = #DE0482;
      } else if (rand == 2) {
        mycolor = #115990;
      } else if (rand == 3) {
        mycolor = #11445E;
      } else if (rand == 4) {
        mycolor = #5EBFD9;
      } else if (rand == 5) {
        mycolor = #252D62;
      }
      mycolor = color(mycolor, 150);
    }
  }
}
