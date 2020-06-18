package cosc250.boids

import scala.collection.mutable

object Simulation {

  /** Wrap width of the simulation. ie, for any Boid, 0 <= x < 640 */
  val width = 640

  /** Wrap height of the simulation. ie, for any Boid, 0 <= y < 480 */
  val height = 480

  /** How many frames of the simulation to hold */
  val frameMemory = 60

  /** How many boids to start with in the simulation */
  val numBoids = 150

  /** When the wind is blowing, how strongly it blows */
  val windStrength = 0.02

  /** When the boids are startled, the strength of the vector that is applied to each of them */
  val startleStrength: Double = Boid.maxSpeed


  /** QUEUE **/

  /** A mutable queue containing the last `frameMemory frames` */
  val queue: mutable.Queue[Seq[Boid]] = mutable.Queue.empty[Seq[Boid]]

  /** The current frame */
  // Function that retrieves the current frame on the queue (the last frame in the queue).
  def current: Seq[Boid] = queue(queue.length-1)

  /** Pushes a state into the queue */
  def pushState(boids: Seq[Boid]): Seq[Boid] = {

    // Enqueue the Seq[Boid] boids
    queue.enqueue(boids)

    // Drops a frame from the queue if we've reached the maximum number of frames to remember
    if (queue.lengthCompare(frameMemory) > 0) queue.dequeue()

    boids
  }

  /** Called by the Action Replay button to jump back in the memory buffer */
  def resetQueue(): Seq[Boid] = {

    // Jump back to the start of the queue and push that frame.
    pushState(queue(0))
  }


  /** STARTLE FEATURE **/

  // Applies the startleStrength to the unstartled Boids position vector.
  val startleFunction: Boid => Vec2 = unstartledBoid => unstartledBoid.position.*(startleStrength)

  /**
    * A function that will run for the next frame only over each boid in the system,
    * producing an acceleration vector to add to a Boid
    */
  var oneTimeFunction: Option[Boid => Vec2] = None

  /**
    * Resets the events that should occur one time only
    */
  def resetOneTimeEvents():Unit =   {

    // Resets the oneTimeFunction to None after the Boids are Startled.
    oneTimeFunction = None
  }


  /**  WIND FEATURE **/

  /** The wind -- an optional acceleration vector */
  var wind: Option[Vec2] = None

  /**
  * Sets a wind blowing at windStrength, at this angle.
  * Note that a northerly wind blows **from** the north, so we multiply the vector by -1.
  */
  def setWindDirection(theta:Double):Unit = {
    /**
      * Update the wind variable with the windstrength and approriate theta.
      * If a wind direction is selected then wind is defined and the appropriate vector is
      * applied to the Boids velocity vector.
      */
    val windDefined:Vec2 = Vec2.fromRTheta(windStrength,theta).*(-1)

    wind = Option(windDefined)
  }


  /** BOID INSERTION FEATURE **/

  /** A container that can hold a boid to add on the next frame */
  var insertBoid: Option[Boid] = None



  /** Called by a click to the canvas, to say that in the next frame, a boid should be inserted */
  def pushBoid(b: Boid): Unit = {

    insertBoid = Some(b)

    /**
      * If insertBoid is defined then that boid is appended to the current frame and the Seq[Boid]
      * is pushed onto the queue. Otherwise, insertBoid is None.
      */
    pushState(current.appended(insertBoid.getOrElse(Boid(Vec2(0,0),Vec2(0,0)))))
  }


  /** BOID GENERATION AND UPDATES **/

  /** Generates boids in the centre of the simulation, moving at v=1 in a random direction */
  def explosionOfBoids(i: Int): Seq[Boid] = {

    //Generate new boids by mapping each boid to the appropriate position and velocity vectors.
    (1 to i).map{ _ => Boid(Vec2(width / 2, height / 2), Vec2.randomDir(1)) }
  }

  /** Generate the next frame in the simulation */
  def update(): Seq[Boid] = {

    // Map every boid in the current sequence to an updated boid.
    val boidUpdate:Seq[Boid] = current.map( oldBoid =>

      /** If the startle button is clicked then the oneTimeFunction is defined.
       *  Then the boid is updated by calling the Boid.update function, with startleFunction
       *  force applied to the oldBoid position vector and the velocity is set to a velocity of random
       *  magnitude with a force of startle strength.
       * */
      if(oneTimeFunction.isDefined)

        oldBoid.update( startleFunction.apply(oldBoid) , Vec2.randomDir(startleStrength))
      else

      /**
        * If the oneTimeFunction() is not defined then the boids are updated by calling the
        * Boid.update function, with the flock of the current frame as the acceleration
        * argument. The velocity vector is updated according to the Boid.update function and
        * wind if it is defined.
        */
        oldBoid.update(oldBoid.flock(current),  wind.getOrElse(Vec2(0,0)))
    )

    //Reset of the oneTimeFunction() to None.
    resetOneTimeEvents()

    // Push the updated Seq[Boid] onto the frame.
    pushState(boidUpdate)
  }
}