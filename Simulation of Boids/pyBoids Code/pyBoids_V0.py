"""
********************************************************************************
Basic Boids whos behavior is modeled by the three boid rules_
    - Cohesion -> Moving closer towards center of Mass of boids neightborhood
    - Repulsion -> Move further, away of center of Mass of boids neightborhood
                   to avoid crowding local flockmates
    - Alignement -> " Move to Orientation: steer towards average heading 
                    " of local flockmates "
********************************************************************************
Author: David C. Alvarez - Charris
Contact: david13.ing@gmail.com

Created on Wed Jan 13 14:17:01 2016

Based on Ben Dowling's code of:
"Boid implementation in Python using PyGame - http://www.coderholic.com/boids/
********************************************************************************
"""

import sys, pygame, random, math
import time #
pygame.init()

# Static Parameters
noBoids = 50
boids = []
scenarioSize = width,height = 800,600 #scenario dimensions

# Dynamic parameters(evolved)    
neighRadii = 100 # Vecinity which each robots checks to apply behavioral rules
maxVel = 10.0  # Max. delta speed that can be performed on each time step
crowdingThr = 15 # how close can agents get one an other

# Behavioral weights
cohW = 0.8/100 # for Cohesion -> 1%
repW = 22.0/100 # for Repulsion -> 20%
alignW = 10.0/100 # for alignment -> 20%


" Class containing all the Boid behaviors "
class Boid: 

    def __init__ (self):
        self.x = random.randint(0,width)
        self.y = random.randint(0,height)
        self.velX = float(random.randint(1,maxVel)) /maxVel
        self.velY = float(random.randint(1,maxVel)) /maxVel


    " Compute (Euclidian) distance from self - to - 'boid' "
    def distance (self,boid):
        distX = self.x - boid.x
        distY = self.y - boid.y
        return math.sqrt(distX*distX + distY*distY)


    " Move closer: towards center of Mass of boids neightborhood"
    def cohesion(self,boids):
        #Calculate average distances to other boids
        avgX = 0
        avgY = 0

        for boid in boids:
            avgX += (self.x - boid.x)
            avgY += (self.y - boid.y)

        # As an analogy, this calculation is  like obtaining the center of mass (x,y) 
        # among the neighborhood members, BUT w.r.t. to the boid of interest (self)
        # current position (not the CoM with respect to the abosolute zero) ...        
        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        # ... and then pushing the boid of interest (self) towards the Center of mass 
        # (CoM) to produce the cohesion behavior. The direction (sign of dispalcement)
        # in which the boid has to be moved to approach the CoM is computed ->
        # currentPos = (currentPos - CoM) , for its X and Y position
        self.velX -= (avgX*cohW) 
        self.velY -= (avgY*cohW) 


    " Move further: away of center of Mass of boids neightborhood "
    " to avoid crowding local flockmates"
    def repulsion(self,boids):
        # Vector which added to currentPosition moves the boid away from those near it
        escapeTX = 0 # X - escape Trajectory
        escapeTY = 0 # Y - escape Trajectory
        isCrowed = 0 # bool: 0-> No agent extremely close ; 1-> agents to close

        # Check for crowded agents
        for boid in boids:
            distance = self.distance(boid)

            if distance < crowdingThr:
                isCrowed = 1
                # Accumulating directions (and mag.) of Escape trajectory vector
                escapeTX += self.x - boid.x
                escapeTY += self.y - boid.y

        # Compute new velocity   
        if isCrowed:
            self.velX += (escapeTX*repW) 
            self.velY += (escapeTY*repW) 


    " Move to Orientation: steer towards average heading of local "
    " flockmates "
    def alignment(self,boids):
        #Calculate average speed to other boids
        avgX = 0
        avgY = 0

        for boid in boids:
            avgX += boid.velX
            avgY += boid.velY

        # As an analogy, this calculation is  like obtaining the center of mass (x,y) 
        # among the neighborhood members, BUT w.r.t. to the boid of interest (self)
        # current position (not the CoM with respect to the abosolute zero) ...        
        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        # ... and then pushing the boid of interest (self) towards the Center of mass 
        # (CoM) to produce the cohesion behavior. The direction (sign of dispalcement)
        # in which the boid has to be moved to approach the CoM is computed ->
        # currentPos = (currentPos - CoM) , for its X and Y position
        self.velX += (avgX*alignW) 
        self.velY += (avgY*alignW) 


    "Perform movement: based on boids velocity (determined by the behavioral rules)"
    def move(self):
        # for each timeStep of evaluyating the rules, theres a maximum (saturation) displacemente
        # value (boids cant move instantly from one place to the other, its got to me smooth)
        if abs(self.velX) > maxVel or abs(self.velY) > maxVel:
            scaleFactor = maxVel / max(abs(self.velX), abs(self.velY))
            self.velX *= scaleFactor
            self.velY *= scaleFactor
        
        self.x += self.velX
        self.y += self.velY


" Main Code "       
def main(): #(argv):    
    # Scenario and Boid loading
    scenario = pygame.display.set_mode(scenarioSize)
    boidIm = pygame.image.load("angryBirdS.png")
    boidImrect = boidIm.get_rect()    

    # initialize Boids, with a random X,Y position
    for i in range(noBoids):
        boids.append(Boid())  


    while 1:
        # Check for any event which exits the simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
            

        # Compute neighbooring boids
        for boid in boids:# for each boid
            neighBoids = []# list which stores neighboors
            
            for otherBoid in boids:# check its distance with the others
                # Dont check the boids distance with its self                
                if otherBoid == boid: continue
                
                dist2neigh = boid.distance(otherBoid)
                if dist2neigh < neighRadii:
                    neighBoids.append(otherBoid) # add to list
            
            # Apply the three behaviors based on current neighborhood (if existant)
            if len(neighBoids) > 1: 
                boid.cohesion(neighBoids)
                boid.repulsion(neighBoids)
                boid.alignment(neighBoids)
            
            # Ensure boids  stay within the screen space -> if they are to close
            # simply change the Direction of the velocity vector, and change
            # its magnitud by a random factor between [0.0,1.0]
            border = 25 # how close from the borders does one wants the boids to get
            bordFact = 0.5
            if boid.x < border and boid.velX< 0:
                boid.velX = -boid.velX*random.random()*bordFact #generates val in [0.0,1.0]
                
            if boid.x > (width - border) and boid.velX> 0:
                boid.velX= -boid.velX* random.random()*bordFact
                
            if boid.y < border and boid.velY < 0:
                boid.velY = -boid.velY * random.random()*bordFact
                
            if boid.y > height - border and boid.velY > 0:
                boid.velY = -boid.velY * random.random()*bordFact
                
            # Simulaion time step -> Update boids position vector
            boid.move()
        
        # Simulaion time step -> Update virtual boids display
        sceneColor = 255,128,0
        scenario.fill(sceneColor) # fill scenario with RGB color
        for boid in boids:
            boidRect = pygame.Rect(boidImrect) #create boids square based on image size
            boidRect.x = boid.x# stablish its position
            boidRect.y = boid.y
            scenario.blit(boidIm,boidRect) #draw one image onto another

        
        pygame.display.flip()
        pygame.time.delay(10)
            
if __name__ == "__main__":
    main()# (sys.argv)


