  let words = ["climate","AGW","gas","environment","USHCN","INDC","energy","temperature","carbon","pollution","co2","science","politics","health","Earth","emission","weather","fossil","fuel","sea-level rise","COP","UNFCCC","IPCC","PPM","methane","mitigation","warm","degre","cool","dioxid","barrel","oil","antarctic","atmosphere","glacier","melt","antarctica","medieval","palaeo","turbin","renew","wind","megawatt","hydrogen","reactor","nuclear","green","cyclone","storm","hurricane","scheme","cultivar","endanger","coral","phytoplankton","ozon","extinct","bear","polar","vehicle","electric","car","millenni","adapt","mercuri","flood","cloud", "ratif","treati","consensus","alarmist","develop","recycle","impact","conservation","forest","EPA","acid","species","simulation","EIA","CLF","GHG","calcification","RGGI","NHTSA","MGP","NAAQ","NDVI","disease","VMT","USHCN","integrity",];

  let typo = [];

  function preload() {
    typo.push(loadFont("assets/Compagnon.otf"));
    typo.push(loadFont("assets/Director_r.otf"));
  }

  const c1 = "#B8A47A";
  const c2 = "#90FF94";
  let color = [c1, c2];

  boids = [];

  function setup() {
    createCanvas(720, 576);
    background(32, 33, 79);

    // Add an initial group of boids into the system
    for (let i = 0; i < 40; i++) {
      let b = new Boid(
        width / 2,
        height / 2,
        random(words),
        random(color),
        random(10, 25),
        random(typo)
      );
      boids.push(b);
    }
  }

  function draw() {
    background(32, 33, 79, 0.5);
    //Draw boids
    for (let i = 0; i < boids.length; i++) {
      boids[i].run(this.boids);
    }
    //Erase boids if the number of boids is over 40
    if (boids.length > 40) {
      boids.splice(0, 1);
    }
  }

  // Add a new boid into the System
  function mousePressed() {
    for (let i = 0; i < 2; i++) {
      let b = new Boid(
        mouseX,
        mouseY,
        random(words),
        random(color),
        random(20, 25),
        random(typo)
      );
      boids.push(b);
    }
  }

  // Boid class
  // Methods for Separation, Cohesion, Alignment added

  function Boid(x, y, mot, teinte, size, typo) {
    this.acceleration = createVector(0, 0);
    this.velocity = createVector(random(-1, 1), random(-1, 1));
    this.position = createVector(x, y);
    this.r = 3.0;
    this.maxspeed = 3; // Maximum speed
    this.maxforce = 0.05; // Maximum steering force
    this.mot = mot; // Word of each the boid
    this.teinte = teinte; // Color of the boid
    this.size = size; // Size of the typo
    this.typo = typo; // Typo of the boid
  }

  Boid.prototype.run = function (boids) {
    this.flock(boids);
    this.update();
    this.borders();
    this.render();
  };

  Boid.prototype.applyForce = function (force) {
    // We could add mass here if we want A = F / M
    this.acceleration.add(force);
  };

  // We accumulate a new acceleration each time based on three rules
  Boid.prototype.flock = function (boids) {
    let sep = this.separate(boids); // Separation
    let ali = this.align(boids); // Alignment
    let coh = this.cohesion(boids); // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.4);
    ali.mult(1.0);
    coh.mult(1.0);
    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
  };

  // Method to update location
  Boid.prototype.update = function () {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
  };

  // A method that calculates and applies a steering force towards a target
  Boid.prototype.seek = function (target) {
    let desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(this.maxspeed);
    // Steering = Desired minus Velocity
    let steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxforce); // Limit to maximum steering force
    return steer;
  };

  Boid.prototype.render = function () {
    fill(this.teinte);
    stroke(32, 33, 79);
    push();
    translate(this.position.x, this.position.y);
    textFont(this.typo);
    textSize(this.size);
    text(this.mot, 0, 0);
    pop();
  };

  // Wraparound
  Boid.prototype.borders = function () {
    if (this.position.x < -this.r) this.position.x = width + this.r;
    if (this.position.y < -this.r) this.position.y = height + this.r;
    if (this.position.x > width + this.r) this.position.x = -this.r;
    if (this.position.y > height + this.r) this.position.y = -this.r;
  };

  // Separation
  // Method checks for nearby boids and steers away
  Boid.prototype.separate = function (boids) {
    let desiredseparation = 30.0;
    let steer = createVector(0, 0);
    let count = 0;
    // For every boid in the system, check if it's too close
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if (d > 0 && d < desiredseparation) {
        // Calculate vector pointing away from neighbor
        let diff = p5.Vector.sub(this.position, boids[i].position);
        diff.normalize();
        diff.div(d); // Weight by distance
        steer.add(diff);
        count++; // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(this.maxspeed);
      steer.sub(this.velocity);
      steer.limit(this.maxforce);
    }
    return steer;
  };

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  Boid.prototype.align = function (boids) {
    let neighbordist = 20;
    let sum = createVector(0, 0);
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      if (d > 0 && d < neighbordist) {
        sum.add(boids[i].velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.normalize();
      sum.mult(this.maxspeed);
      let steer = p5.Vector.sub(sum, this.velocity);
      steer.limit(this.maxforce);
      return steer;
    } else {
      return createVector(0, 0);
    }
  };

  // Cohesion
  // For the average location of all nearby boids, calculate steering vector towards that location
  Boid.prototype.cohesion = function (boids) {
    let neighbordist = 50;
    let sum = createVector(0, 0); // Start with empty vector to accumulate all locations
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      if (d > 0 && d < neighbordist) {
        sum.add(boids[i].position); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return this.seek(sum); // Steer towards the location
    } else {
      return createVector(0, 0);
    }
  };

