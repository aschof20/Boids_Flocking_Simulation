package cosc250.boids

/**
  * A boid (bird-oid). It has a position and a velocity.
  *
  *
  * https://processing.org/examples/flocking.html
  */
case class Boid(position:Vec2, velocity:Vec2) {

  /**
    * Calculates an acceleration vector that will cause it to maintain a minimum
    * separation from its closest neighbours
    * This steer is limited to maxForce
    */
  def separate(others:Seq[Boid]):Vec2 = {

    // Filter the cohort of Boids in the Seq[Boid] that are within the desiredSeparation.
    val boidsTooClose:Seq[Boid] = within(others, Boid.desiredSeparation)

    /**
      * Calculate a vector pointing away from the boid, which is weighted by the distance.
      * Weighting adds more weight to closer boids. Finally, sum all the vectors together.
      */
    val pointAway:Vec2 = boidsTooClose.foldLeft(Vec2(0,0)) { (sumSeparate, otherBoid) =>

      sumSeparate.+((position - otherBoid.position).normalised./(distance(position, otherBoid.position)))
    }

    // Get the average of the vectors from pointAway and calculate the steering vector.
    val steer:Vec2 = if(boidsTooClose.nonEmpty) {

      // Average of the pointing away vectors, limited by maxForce.
      pointAway./(boidsTooClose.size).limit(Boid.maxForce)

      // Calculate the final steering vector i.e Steering = Desired - Velocity.
      pointAway.normalised.*(Boid.maxSpeed).-(velocity)
    } else {

      Vec2(0,0)
    }
    steer
  }

  /**
    * Calculates an acceleration vector that will cause it align its direction and
    * velocity with other birds within Boid.neighbourDist
    * This alignment force is limited to maxForce
    */
  def align(others:Seq[Boid]):Vec2 = {

    // Filter the cohort of Boids in the Seq[Boid] that are within the neighBourDist.
    val neighbourBoids:Seq[Boid] = within(others, Boid.neighBourDist)

    /**
      * Calculate an acceleration vector that is the sum of the velocities of all
      * neighbouring boids.
      */
    val alignDirection:Vec2 = neighbourBoids.foldLeft(Vec2(0,0)) { (sumAlign, otherBoid) =>

      sumAlign.+(otherBoid.velocity)
    }

    // Calculate the average of the neighbouring boids velocity vectors and get the steering vector.
    val steer:Vec2 = if(neighbourBoids.nonEmpty){

      // Average of the neighbour velocities normalised.
      val avgVelocity:Vec2 = alignDirection./(neighbourBoids.size).normalised

      // Calculate the final steering vector.
      avgVelocity.*(Boid.maxSpeed).-(velocity.limit(Boid.maxForce))
    }else {

      Vec2(0,0)
    }
      steer
  }
  
  /**
    * Calculates an acceleration that will steer this boid towards the target.
    * The steer is limited to maxForce
    */
  def seek(targetPos: Vec2):Vec2 = {
    
    // Vector pointing to the target position, that is normalised and scaled to the max speed.
    val desired:Vec2 = targetPos.-(position).normalised.*(Boid.maxSpeed)
    
    // Steering of the boid towards the target position limited by the max force
    val steer:Vec2 = desired.-(velocity).limit(Boid.maxForce)
   
    steer
  }

  /**
    * Calculates an acceleration that will keep it near its neighbours and maintain
    * the flock cohesion.
    */
   def cohesion(others: Seq[Boid]):Vec2 = {

    //Filter the cohort of boids that are neighbours.
    val neighbourBoids:Seq[Boid] = within(others, Boid.neighBourDist)

    //Calculate an acceleration vector that will steer the neighbourBoids to the target position.
    val steer:Vec2 = neighbourBoids.foldLeft(Vec2(0,0)){ (sumCohesion, otherBoid) =>

      // Add all the position vectors from neighbourBoids.
      sumCohesion.+(otherBoid.position)

      /**
        * Call seek on the average neighbourBoid vector to calculate the force vector so birds flock to a
        * target position.
        */
      seek(sumCohesion./(neighbourBoids.size))
    }
    steer
   }

  /**
   * Calculates a flocking acceleration that is a composite of its separation,
   * align, and cohesion acceleration vectors.
   */
  def flock(boids: Seq[Boid]):Vec2 = {

    val separateVector:Vec2 = separate(boids)     // Separation vector for Seq[Boid]
    val alignVector:Vec2 = align(boids)           // Alignment vector for Seq[Boid]
    val cohesionVector:Vec2 = cohesion(boids)     // Cohesion vector for Seq[Boid]

   // Acceleration vectors are added together and limited by maxForce
   separateVector.+(alignVector).+(cohesionVector).limit(Boid.maxForce)
  }

 /**
   * Produces a new Boid by adding the boid's velocity to its position, and adding
   * the acceleration vector to the boid's velocity. Note that there is no division
   * by timestep -- it's just p = p + v, and v = v + a
   *
   * Also note that we don't apply the limiting on maxForce in this function -- this is
   * so that the startle effect can dramatically perturb the birds in a way they would
   * not normally be perturbed in flight. Instead, limit maxForce in the flock function
   * (or the functions it calls)
   *
   * We do, however, limit a boid's velocity to maxSpeed in this function. But we do it
   * *before* we add the influence of the wind to the boid's velocity -- it's possible
   * to fly faster downwind than upwind.
   */
   
 def update(acceleration:Vec2, wind:Vec2):Boid = {

   // Add the acceleration vector to the velocity vector and limit the vector to maxSpeed.
   val updateVelocity:Vec2 =  velocity.+(acceleration).limit(Boid.maxSpeed)

   // Add wind to the updated velocity vector to get the new velocity.
   val newVelocity:Vec2 = updateVelocity.+(wind)

   //Add the velocity vector to the position vector to get the new position vector.
   val newPosition:Vec2 = position.+(velocity)

   //Create new Boids with new position vector that wraps around the plane and a new velocity vector.
   Boid(Vec2(wrapX(newPosition.x), wrapY(newPosition.y)), newVelocity)
 }

  def wrapX(x:Double):Double = {
    if (x > Boid.maxX) x - Boid.maxX else if (x < 0) x + Boid.maxX else x
  }

  def wrapY(y:Double):Double = {
    if (y > Boid.maxY) y - Boid.maxY else if (y < 0) y + Boid.maxY else y
  }

  /**
    * Function that calculates the distance between two Boids, using Pythagoras' theorem.
    *
    * @param vecOne - the position vector of a boid.
    * @param vecTwo - the position vector of another boid.
    */
  def distance(vecOne: Vec2, vecTwo: Vec2):Double = {

    val dx = vecOne.x.-(vecTwo.x) // x-value delta between vecOne and vecTwo.
    val dy = vecOne.y.-(vecTwo.y) // y-value delta between vecOne and vecTwo.

    //Applying Pythagoras' theorem.
    Math.sqrt(Math.pow(dx, 2)+ Math.pow(dy, 2))
  }

  /**
    * Function that filters a Seq[Boid] that are within an arbitrary distance and the distance is greater
    * than 0. This is used multiple times so it gets its own function.
    *
    * @param boids - a Seq[Boid].
    * @param dist - the arbitrary distance between two boids to be tested.
    */
  def within(boids: Seq[Boid], dist: Double):Seq[Boid] = {

    boids.filter( other => distance(position, other.position) < dist && distance(position, other.position) > 0)
  }

}

object Boid {

   /** How far apart the boids want to be */
   val desiredSeparation = 25

   /** Maximum flying velocity of a boid */
   val maxSpeed = 2

   /** maximum accelaration of a boid */
   val maxForce = 0.03

   /** Other boids within this range are considered neighbours */
   val neighBourDist = 50

   /** Wrap width of the simulation. ie, for any Boid, 0 <= x < 640 */
   def maxX:Int = Simulation.width

   /** Wrap height of the simulation. ie, for any Boid, 0 <= y < 480 */
   def maxY:Int = Simulation.height
}
