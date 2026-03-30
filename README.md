# AGV-Task-2-Nakul
This is my GITHUB repo that has my work on task of AGV to build a simulator that avoids dynamic obstacles.

# Dynamic Obstacle Avoidance

### Components:

1. **Waypoint generation (static map)**
2. **Tracking controller (Pure Pursuit)**
3.  **Obstacle avoidance**
4. **Dynamic obstacle handling (momentum-based)**
5. **Robustness layer (noise, stuck detection, safety)**

---

# 0. Key Implementations I tried to put

### 1. NC3-style local triangulation

### 2. Momentum-based obstacle weighting (Using estimated moving averages to assign weighst to lidar rays distance data )

### 3. Checking if the AGV gets stuck or not, and then we just skip the waypoint it was supposed to go to

### 4.  braking in advance when danger is seen or slowing down



# 1. Waypoint Generation (Subtask 1 Core)

### Implemented Approach: Grid Sampling + Local Triangulation (NC3 logic)

Instead of directly applying the Delaunay triangulation, I used:

### Step 1: Uniform grid sampling

```
grid =8 → samplepointsin [-0.8,0.8]
```

- Points inside obstacles are removed using:

```
contains(curmap[k],point(wx,wy))
```

### Step 2: Local triangle formation (NC3-inspired)


For each free point:

- Find **2 nearest neighbours**
- Form triangle
- Compute centroid:

C=Pi+Pn1+Pn2/3

 This is how I tried making a triangular mesh

---

### Step 3: Downsampling (Coz i got way to many waypoint and it became jerky)

```
step =centroids.size()/30
```

- Keeps ~30 waypoints
- Prevents overfitting / oscillations

---

### Step 4: Greedy ordering (Nearest Neighbour)

- Starts from current position
- Picks closest unused waypoint iteratively

---

## Scrapped Ideas (Planning Stage)

### Full Delaunay triangulation

### or taking points of the edges of the obstacles and doing nc3 for that

---

# 2. Pure Pursuit Controller (Tracking)

Track a point ahead on the path instead of the immediate waypoint.

issue: even though the waypoint are not inside the obstacles wont ensure that the path the agv travels is also out of the obstacle

### Implementation:

Lookahead distance: Ld=0.15+1.5∣v∣

- Transform target to vehicle frame:

]

ylocal=sin(−θ)dx+cos(−θ)dy

- Steering:

δ= 2y/L^2d

## Scrapped Controller Ideas

### PID-only steering

### Follow-the-gap (global use)

---

# 3. Obstacle Avoidance (Subtask 2 Core)

## Sensor Processing

### LiDAR distance:

di=sqrt{(x_i - x)^2 + (y_i - y)^2}

---

## EMA Filtering (Noise Handling) (Used Finance stock logic of moving averages  to decide weights on the rays lengths)


alpha =0.2

### Purpose:

- Smooth noisy LiDAR
- Reduce jitter

---

## Momentum-Based Obstacle Detection

momentum=prevEMA−current

### Interpretation:

- Positive → obstacle approaching
- Negative → obstacle moving away

### Weighting:

- Approaching → increase importance
- Receding → decrease importance

 This mimics **velocity estimation without explicit tracking**

---

## Threat Field Construction

For each ray:

w=((D−d)/(D))^2⋅dyn_weight

Additional:

- **Frontal boost** using cosine
- Left/right separation

---

### Result:

```
avoid =left_threat-right_threat
```

---

## Scrapped Avoidance Ideas

### Velocity Obstacles (VO)

---

# 4. Navigation + Avoidance

### Danger metric:

danger=D−min_dist/D

### Final steering:

steer=(1−danger)⋅goal+danger⋅avoid

---

## Braking in Advanced after seeing danger

danger_rate=danger−prev_danger

anticipation=max⁡(0,danger_rate)

This basically Slows down when danger is increasing

---

# 5. Control Smoothing

### Steering smoothing:

steer=0.72⋅prev+0.28⋅new

### Acceleration smoothing:

a=0.85⋅prev+0.15⋅target

Tried to Prevent oscillations or jittery mvement by restricting immediate movement but wasnt able to 

In advance breaking after seeing danger

---

# 6. Dynamic Obstacles (Simulator Setup)

I created **3 motion types**:

1. Sinusoidal (x-direction)
2. Sinusoidal (y-direction)
3. Circular motion

x=x0+Asin(ωt)

y=y0+Acos(ωt)

Basically oscillate in x and y direction with different speeds
