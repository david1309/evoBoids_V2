"""
********************************************************************************
 Boids whos behavior is modeled boid rules and additional reactions: 
    * Behaviors [applied to (Casual Agents -> rainbow color, self.type == 0)]
        - Cohesion -> Moving closer towards center of Mass of boids neightborhood
        - Repulsion -> Move further, away of center of Mass of boids neightborhood
                   to avoid crowding local flockmates
        - Alignement -> " Move to Orientation: steer towards average heading 
                    " of local flockmates "

    * Special Agents:
        - Leader Following -> Boids have a strong atraction to a particular agent (Leader -> Blue, self.type>=1)
        - Predator run away -> Boids run away from a particular agent (Predator -> Red, -99<=self.type<=-1)

    * Others:
        - Obstacle avoidance -> Boids evade and split up to avoid runing into bostacle (Obstacle-> Green, self.type<=-100)

 * Simulation Controls:
    - left mouse click: Created new controlled special agnets (after each click, agent will conmuted between leader/predator)
    - center mouse click: Created an autonomous (computer controlled) leader/predator particular
    - right mouse click: Draw obstacles
    - Left + Right click: Erase all the crated agents and obstacles

 * Simulation Argument Call:
    - Arguemtns will +/- to default values, MUST be 0<=integers<=9 and be accompanied by the +/- 
    - Argument order: leadersAtractionWeight, predatorsElusionWeight, obstacleFear, specialAgentsRadiusofDetection, 
                      cohesionWeight, repulsionWeight, alignmentWeight, 
                      autoAgentsMode (-1:Auto.Predator,+1: Auto.Leader,0:Auto.Leader and Auto.Predator)        
    -Scheme of User Command: python pyBoids_v1.py +0,+0,+0,+0,+0,+0,+0,+1"

********************************************************************************
    Author: David C. Alvarez - Charris
    Contact: david13.ing@gmail.com

    Created on Fri Jan 15 17:30:18 2016

    Based on Andrew Davison 2013 work 
    http://www.doc.ic.ac.uk/~ajd/SimplePython/swarm.py
********************************************************************************

"""

import sys, pygame, random, math
import time # time.sleep([s]) 
pygame.init()

"Visualization variables"
scenarioSize = width,height = 800,600 #scenario dimensions
deltC = 0 #1 # how much does the BG color change on each interation
sceneColorOri = (255,255,255) # (100,100,255) Initial background color
varC = [1,0,0] # variable used to increase/decrase each [R,G,B] color

"Static Parameters"
noBoids = 60 # 50
boids = [] # list containing all the agents
# simulation might runs faster/slower depending on amount of agents. Changing damping factor easily
# adjust speed and other important factor for a nice simulation 
damp = 1.0
spread = 1.0 # Determines the (x,y) range, among which initial position are going to be randonmly stablished

"Dynamic parameters"   
neighRadii = 150.0 # Vecinity which each agents checks to apply behavioral rules
maxVel = 10.0*damp  #10 Max. delta speed that can be performed on each time step
crowdingThr = 10.0 # how close can agents get one an other before "repulsion behavior" is activated

# Behavioral weights
cohW = 4.0/100 # for Cohesion 
repW = 23.0/100 # for Repulsion 
alignW = 20.0/100 # for alignment 

"Special Agents and obstacles params. [Corresponds to self.type != 0]"
# when Special Agents are automous (controlled by code), determines the maximum change in speed at each time step
maxDeltaVel = 4.0*damp
leaderW = 20.0 #10 How much more cohesion and less repulsion will Leader have
predatorW = 12.0 # How much more repulsion will predator have
avoidW = 11 #13.0 # How much more repulsion (how far away) will agents evade obstacles
specialRFact = 20.0 #15 Factor which increases the Radii at which Special Agents are considerder neighbors
# 0: Created both, leader/predator Auto.Agents ; 1/-1: Creadte Auto.leader/Auto predator agents
autoAgents = 0 


" Class containing all the Boid behaviors "
class Boid: 

    def __init__ (self, type):
        # Casual Agent : set pos/vel randonmly ; Special Agents: pos == center of screen and vel == maxSpeed %
        self.x = random.randint(-spread*width,(spread+1)*width)*(type==0) + (width/2)*(type!=0)
        self.y = random.randint(-spread*height,(spread+1)*height)*(type==0)  + (height/2)*(type!=0)
        self.velX = (float(random.uniform(-maxVel,maxVel)) /maxVel)*(type==0) + (maxVel/3)*(type!=0)
        self.velY = (float(random.uniform(-maxVel,maxVel)) /maxVel)*(type==0) + (maxVel/3)*(type!=0)
        self.type = type          


    " Compute (Euclidian) distance from self - to - 'boid' "
    def distance (self,boid):
        distX = self.x - boid.x
        distY = self.y - boid.y
        return math.sqrt(distX*distX + distY*distY)


    " Move closer: towards center of Mass of boids neightborhood"
    def cohesion(self,boids):
        # Average distnace variables
        avgX = 0
        avgY = 0

        # comment at: C1!? ->When to many agents follow the leader, the agents start
        # having more "cohesion force" than the leader. They end up prefering being 
        # together over following the leader. This command forces the increment of the leaders 
        # "cohesion force" so that he can continue leading, despite the number of agents on the group

        for boid in boids:
            fact = 1.0 # used to modify Special Agents "cohesion force"
            if boid.type > 0: fact = leaderW #  leader positive reinforcement
            # if len(boids)>10 and type>0: fact = leaderW*leaderW # <-- C1!?                               
            elif boid.type < 0: fact = 0 # predator negative reinforcement

            # Tab this 2 lines under the first 'if' and increase cohFact >=2 0 to have agents only follow LEADER
            # Calculate average distances to other boids
            avgX += (boid.x - self.x)*fact
            avgY += (boid.y - self.y)*fact            
                       
        # As an analogy, this calculation is  like obtaining the center of mass (x,y) 
        # among the neighborhood members, BUT w.r.t. to the boid of interest (self)
        # current position (not the CoM with respect to the abosolute zero) ...        
        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        # ... and then pushing the boid of interest (self) towards the Center of mass 
        # (CoM) to produce the cohesion behavior. The direction (sign of dispalcement)
        # in which the boid has to be moved to approach the CoM is computed ->
        # currentPos = (currentPos - CoM) , for its X and Y position
        self.velX += (avgX*cohW) 
        self.velY += (avgY*cohW) 


    " Move further: away of center of Mass of boids neightborhood "
    " to avoid crowding local flockmates"
    def repulsion(self,boids):
        # Vector which added to currentPosition moves the boid away from those near it
        escapeTX = 0 # X - escape Trajectory
        escapeTY = 0 # Y - escape Trajectory
        isCrowed = 0 # bool: 0-> No agent extremely close ; 1-> agents to close
        
        # comment at: C2!? -> When there are to many agents in the leaers cluster,
        # make agents repude the Leader from a greater radii

        # Check for crowded agents
        for boid in boids:
            distance = self.distance(boid)

            fact = 1.0 # used to modify Special Agents "repulsion force"
            if boid.type > 0: fact = leaderW # decreases radii at which leader is repulsed
            # if len(boids)>10 and type>0: fact /=100 # <-- C2!?
            elif boid.type < 0 and boid.type>=-99 : fact = 1/predatorW  # increases radii at which predator is repulsed
            elif boid.type<=-99 : fact = 1.0/(avoidW/3)  # increases radii at which obstacle is avoided

            if distance < crowdingThr/fact:
                isCrowed = 1          
                # Accumulating directions (and mag.) of Escape trajectory vector
                escapeTX += (self.x - boid.x)
                escapeTY += (self.y - boid.y)

        # Compute new velocity   
        if isCrowed:
            self.velX += (escapeTX*repW) 
            self.velY += (escapeTY*repW) 


    " Move to Orientation: steer towards average heading of local "
    " flockmates "
    def alignment(self,boids):
        #Calculate average speed of the neighboring boids
        avgX = 0
        avgY = 0

        for boid in boids:  
            fact = 1.0 # used to modify Special Agents "ralignement force"
            if boid.type > 0: fact = 0 # dont follow leaders speed
            elif boid.type < 0 and boid.type>=-99 : fact = 0 # dont follow predator speed  

            avgX += boid.velX*fact
            avgY += boid.velY*fact  

        avgX /= float(len(boids)) 
        avgY /= float(len(boids))  

        self.velX += (avgX*alignW) 
        self.velY += (avgY*alignW) 


    "Perform movement: based on boids velocity (determined by the behavioral rules)"
    def move(self):
        if self.type : # increase special autonomous agent speed            
            self.velX += random.uniform(-maxDeltaVel,maxDeltaVel)
            self.velY += random.uniform(-maxDeltaVel,maxDeltaVel)
    
        # for each timeStep of evaluating the rules, theres a maximum (saturation) displacement
        # value (boids cant move instantly from one place to the other, its got to me smooth)
        if  abs(self.velX) > maxVel or abs(self.velY) > maxVel:
            scaleFactor = maxVel / max(abs(self.velX), abs(self.velY))
            self.velX *= scaleFactor
            self.velY *= scaleFactor

        # Update Agents Position
        if (self.type == 1 or self.type == -1): # Controlled Special Agents
            self.x,self.y = pygame.mouse.get_pos()            
        else:  # Autonomous casual agents                  
            self.x += self.velX
            self.y += self.velY

        # Ensure boids  stay within the screen space -> if they are to close to 
        # the border, then change the Direction of the velocity vector, and change
        # its magnitud by a random factor between [0.0,1.0]
        border = 25 # how close from the borders does one wants the boids to get
        bordFact = 1.5
        if self.x < border and self.velX< 0:
            self.velX = -self.velX*random.random()*bordFact #generates val in [0.0,1.0]
            
        if self.x > (width - border*2) and self.velX> 0:
            self.velX= -self.velX* random.random()*bordFact
            
        if self.y < border and self.velY < 0:
            self.velY = -self.velY * random.random()*bordFact
            
        if self.y > (height - border*2) and self.velY > 0:
            self.velY = -self.velY * random.random()*bordFact 


" Main Code "       
def main(argv): 

    "User custom params (if not provided, DEFAULT params are used)"     
    if len(argv)>1:
        global leaderW, predatorW, avoidW,specialRFact
        global cohW,repW,alignW,autoAgents
        leaderW = leaderW + float(argv[1][0:2]) # How much more cohesion and less repulsion will Leader have
        predatorW = predatorW + float(argv[1][3:5]) # How much more repulsion will predator have
        avoidW = avoidW + float(argv[1][6:8]) # How much more repulsion (how far away) will agents evade obstacles
        specialRFact = specialRFact + float(argv[1][9:11]) # FRadii at which Special Agents are considerder neighbors   
        cohW = cohW + float(argv[1][12:14])  # for Cohesion -> 1%
        repW = repW + float(argv[1][15:17]) # for Repulsion -> 20%
        alignW = alignW + float(argv[1][18:20]) # for alignment -> 20%
        autoAgents = int(argv[1][21:23])

    "Scenario and Boid Params"
    scenario = pygame.display.set_mode(scenarioSize) # Configure pygame window
    sceneColor = [sceneColorOri[0],sceneColorOri[1],sceneColorOri[2]] # stores current BG color
    isControlAgent = 0 # 0: No controlled agents created ; 1: controlled agent already created

    # lists that store controled and special agent object instances (needed in order to delete them when needed)
    controledAgent = []
    specialAgent = []
    avoidPos = [] # list storing the position obstables

    # initialize Casual boids, with a random X,Y position
    for i in range(noBoids):
        boids.append(Boid(0)) # casual agent     

    while 1:
        # Check for any event which exits the simulation
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()

        " User Controls "
        mouseButtons = pygame.mouse.get_pressed()
         
        if mouseButtons[0] and mouseButtons[2]: # Delete all created agents and obstacles
            isControlAgent = 0 
            draw = 0
            if controledAgent : # Destroy Controled Agents
                boids.remove(controledAgent)   
                controledAgent = []       
            if specialAgent : # Destroy Autonomous special agents
                for i in range(len(specialAgent)):
                    boids.remove(specialAgent[i])  
                specialAgent = [] 
            if avoidPos : # Destroy drawed obstacles
                for i in range(len(avoidPos)):
                    avoidPos.remove(avoidPos[0])  
                avoidPos = [] 

            time.sleep(0.25) 

        elif mouseButtons[0]: # Create Controlled special agent 
            if isControlAgent == 0:               
                controledAgent = Boid(1)
                boids.append(controledAgent) # Add to list saving all Boid instances 

            # Conmmute between: Leader and Depredator on each new left click
            controledAgent.type = controledAgent.type-controledAgent.type*isControlAgent*2 
            isControlAgent = 1
            time.sleep(0.15) 

        elif mouseButtons[1]: # Create Autonomous special agents 
            if (autoAgents): specialAgent.append(Boid(2*autoAgents)) # Create Auto. Leader/Predator
            elif(not autoAgents):# Created both special agents
                specialAgent.append(Boid(2))
                specialAgent.append(Boid(-2))
            for i in range(abs(autoAgents)+ 2*(not autoAgents)):
                    boids.append(specialAgent[-1-i])            
            time.sleep(0.15)   

        elif(mouseButtons[2]): # Initialize new "Avoid"  object instance and save its positionn
            avoidPos.append(Avoid(pygame.mouse.get_pos()))
         
        " Compute neighbooring boids and apply ALL behaviors " 
        for boid in boids:# for each boid

            neighBoids = []# list which stores boid neighboors
            
            # CHECK: boid's distance with the other boids
            for otherBoid in boids:
                # Dont check the boids distance with its self                
                if otherBoid == boid: continue
                
                dist2neigh = boid.distance(otherBoid)

                fact = 1.0
                # if the otherBoid is a SpecialAgent, change the radii at which he will
                # be considered a neighbor 
                if otherBoid.type: fact = specialRFact/6

                if dist2neigh < neighRadii*fact:
                    neighBoids.append(otherBoid) # add to neighbors list

            # CHECK: neighboring obstacles
            neighAvoid = [] # list which stores obstacle neighboors
            for avoid in avoidPos:                
                dist2obs = boid.distance(avoid)

                if dist2obs < neighRadii:
                    neighAvoid.append(otherBoid) # add to list

            " Apply the three behaviors based on current neighborhood (if existant) "
            if neighBoids and boid.type == 0 : # Only apply rule to Casual Agents
                boid.cohesion(neighBoids)
                boid.repulsion(neighBoids)
                boid.alignment(neighBoids)

                if neighAvoid: # if current boid has some near obstacle, apply repulsion behavior to it
                    boid.repulsion(avoidPos)         

            " Update boids position vector "
            boid.move()                      
           
        " Simu. Visualization: Update virtual boids display "
        # Update Dynamic Color Background visualization    
        sceneColor = updateBG(sceneColor)        
        scenario.fill(sceneColor) # fill scenario with RGB color

        # Draw obstacles
        if avoidPos: 
            for pos in avoidPos:
                pygame.draw.circle(scenario,(255,255,255),(pos.x,pos.y),9,0)  
                pygame.draw.circle(scenario,(0,255,0),(pos.x,pos.y),7,0)

        # Update Boids visualization
        for boid in boids:
            if boid.type > 0 : # Leaders              
                pygame.draw.circle(scenario, (0,0,200), (int(boid.x), int(boid.y)), 11, 0)
                pygame.draw.circle(scenario, (51,128,255), (int(boid.x), int(boid.y)), 7, 0)
            elif boid.type < 0 :  # Predators              
                pygame.draw.circle(scenario, (0,0,0), (int(boid.x), int(boid.y)), 11, 0)
                pygame.draw.circle(scenario, (255,0,0), (int(boid.x), int(boid.y)), 7, 0)
            else: # Casual Agents
                # Make casual agents blink w/ rainbow like colors
                agentColor = (random.randint(0,240),random.randint(0,240),random.randint(0,240)) 
                pygame.draw.circle(scenario, agentColor, (int(boid.x), int(boid.y)), 4, 3)
                    
        pygame.display.flip()
        pygame.time.delay(10)

" Class which initializes obstacles objects, in a similar way to boid objects."
" In order to be able to reuse class Boid functions, and present the obstacles"
" as a variation of a predator"
class Avoid:
    def __init__ (self,pos):
        self.x = pos[0]
        self.y = pos[1]
        self.type = -100 # obstacles -> self.type <=-100


" Generates the sequence of step that enable the background to change color"
" Increases R -> then G -> then B all until 255 ; then decreases them in the same"
" order to the stablished original (base) color"
def updateBG(sceneColor):
    sceneColor[0] = sceneColor[0] + random.randint(-deltC*(varC[0]==-1),deltC*(varC[0]==1))*(varC[0]==1 or varC[0]==-1)
    sceneColor[1] = sceneColor[1] + random.randint(-deltC*(varC[1]==-1),deltC*(varC[1]==1))*(varC[1]==1 or varC[1]==-1)
    sceneColor[2] = sceneColor[2] + random.randint(-deltC*(varC[2]==-1),deltC*(varC[2]==1))*(varC[2]==1 or varC[2]==-1)
    
    if(sceneColor[0]>255):
        sceneColor[0] = 255 
        varC[0] = 0
        varC[1] = 1
    if(sceneColor[1]>255): 
        sceneColor[1] = 255
        varC[1] = 0
        varC[2] = 1
    if(sceneColor[2]>255): 
        sceneColor[2] = 255
        varC[0] = -1
        varC[2] = 0
    if(sceneColor[0]<sceneColorOri[0] or sceneColor[0]<0 ): 
        sceneColor[0] = sceneColorOri[0]            
        varC[1] = -1
        varC[0] = 0
    if(sceneColor[1]<sceneColorOri[1] or sceneColor[1]<0): 
        sceneColor[1] = sceneColorOri[1]
        varC[2] = -1
        varC[1] = 0            
    if(sceneColor[2]<sceneColorOri[2] or sceneColor[2]<0): 
        sceneColor[2] = sceneColorOri[2]
        varC[2] = 0
        varC[0] = 1

    return sceneColor
            
if __name__ == "__main__":
    main(sys.argv)


