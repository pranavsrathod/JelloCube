/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <algorithm>

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */

void applySingleSpringForce(struct world *jello, int i, int j, int k, const int offset[3], double kElastic, double restLength, struct point *totalForce) {
    int ni = i + offset[0];
    int nj = j + offset[1];
    int nk = k + offset[2];
    if (ni < 0 || ni > 7 || nj < 0 || nj > 7 || nk < 0 || nk > 7) return;

    struct point L, force;
    pDIFFERENCE(jello->p[i][j][k], jello->p[ni][nj][nk], L);
    double length = sqrt(L.x * L.x + L.y * L.y + L.z * L.z);
    double displacement = length - restLength;
    double magnitude = -kElastic * displacement / length;
    pMULTIPLY(L, magnitude, force);
    pSUM(*totalForce, force, *totalForce);
}

void applySpringForces(struct world *jello, int i, int j, int k, struct point *totalForce) {
    // Struct to hold spring type configurations
    struct SpringType {
        const int (*offsets)[3];
        int count;
        double restLengthFactor;
    };

    // Define all spring types in a compact and structured manner
    const int structOffsets[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    const int shearOffsets[20][3] = {
        {1,1,0}, {-1,1,0}, {-1,-1,0}, {1,-1,0}, {0,1,1}, {0,-1,1}, {0,-1,-1}, {0,1,-1},
        {1,0,1}, {-1,0,1}, {-1,0,-1}, {1,0,-1}, {1,1,1}, {-1,-1,1}, {-1,1,-1}, {1,-1,-1},
        {1,1,-1}, {-1,-1,-1}, {-1,1,1}, {1,-1,1}
    };
    const int bendOffsets[6][3] = {{2,0,0}, {-2,0,0}, {0,2,0}, {0,-2,0}, {0,0,2}, {0,0,-2}};

    // Array of spring types
    SpringType springs[] = {
        {structOffsets, 6, 1.0 / 7.0},
        {shearOffsets, 20, sqrt(2) / 7.0},
        {bendOffsets, 6, 2.0 / 7.0}
    };

    // Apply forces for each spring type
    int numSpringTypes = sizeof(springs) / sizeof(SpringType);
    for (int t = 0; t < numSpringTypes; t++) {
        for (int n = 0; n < springs[t].count; n++) {
            applySingleSpringForce(jello, i, j, k, springs[t].offsets[n], jello->kElastic, springs[t].restLengthFactor, totalForce);
        }
    }
}

void applyDampingForces(struct world *jello, int i, int j, int k, struct point *totalForce) {
    struct point dampingForce, velocityDiff, L;
    double length, dotProduct, scalar;

    // Structural Offsets (6 direct neighbors)
    const int structOffsets[6][3] = { 
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1} 
    };

    // Shear Offsets (20 diagonal neighbors)
    const int shearOffsets[20][3] = { 
        {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0},  
        {0, 1, 1}, {0, -1, 1}, {0, -1, -1}, {0, 1, -1},  
        {1, 0, 1}, {-1, 0, 1}, {-1, 0, -1}, {1, 0, -1},  
        {1, 1, 1}, {-1, -1, 1}, {-1, 1, -1}, {1, -1, -1},
        {1, 1, -1}, {-1, -1, -1}, {-1, 1, 1}, {1, -1, 1}  
    };

    // Bend Offsets (6 second-nearest neighbors)
    const int bendOffsets[6][3] = { 
        {2, 0, 0}, {-2, 0, 0}, {0, 2, 0}, {0, -2, 0}, {0, 0, 2}, {0, 0, -2} 
    };

    // Apply damping forces from all types of springs
    for (int type = 0; type < 3; type++) {
        const int (*offsets)[3];
        int numOffsets;
        
        if (type == 0) { offsets = structOffsets; numOffsets = 6; }   // Structural
        else if (type == 1) { offsets = shearOffsets; numOffsets = 20; }   // Shear
        else { offsets = bendOffsets; numOffsets = 6; }     // Bend

        for (int n = 0; n < numOffsets; n++) {
            int ni = i + offsets[n][0];
            int nj = j + offsets[n][1];
            int nk = k + offsets[n][2];

            if (ni < 0 || ni > 7 || nj < 0 || nj > 7 || nk < 0 || nk > 7) continue;

            // Compute L (vector from neighbor to current point)
            pDIFFERENCE(jello->p[i][j][k], jello->p[ni][nj][nk], L);
            length = sqrt(L.x * L.x + L.y * L.y + L.z * L.z);

            if (length == 0) continue;  // Avoid division by zero

            // Compute velocity difference
            pDIFFERENCE(jello->v[i][j][k], jello->v[ni][nj][nk], velocityDiff);

            // Compute dot product: (v_A - v_B) ⋅ L
            dotProduct = velocityDiff.x * L.x + velocityDiff.y * L.y + velocityDiff.z * L.z;

            // Compute damping force magnitude: -k_d * ( (v_A - v_B) ⋅ L ) / |L|
            scalar = -jello->dElastic * (dotProduct / length);

            // Compute final damping force: (scalar * L) / |L|
            pMULTIPLY(L, scalar / length, dampingForce);

            // Accumulate damping force
            pSUM(*totalForce, dampingForce, *totalForce);
        }
    }
}

int detectCollision(struct point p) {
    return (p.x < -2 || p.x > 2 || p.y < -2 || p.y > 2 || p.z < -2 || p.z > 2);
}


void applyCollisionForces(struct world *jello, int i, int j, int k, struct point *totalForce) {
    struct point collisionForce, dampingForce;
    pMAKE(0, 0, 0, collisionForce);  // Initialize force
    pMAKE(0, 0, 0, dampingForce);  // Initialize damping force

    double kCollision = jello->kCollision;  // Read from world file
    double dCollision = jello->dCollision;  // Read from world file

    struct point *p = &jello->p[i][j][k];
    struct point *v = &jello->v[i][j][k];

    // Normal vector to apply force in the correct direction
    struct point normal;
    pMAKE(0, 0, 0, normal);

    // Check X boundaries
    if (p->x < -2.0) {
        normal.x = 1;
        collisionForce.x += kCollision * (-2 - p->x);
    } else if (p->x > 2.0) {
        normal.x = -1;
        collisionForce.x += kCollision * (2 - p->x);
    }

    // Check Y boundaries
    if (p->y < -2) {
        normal.y = 1;
        collisionForce.y += kCollision * (-2 - p->y);
    } else if (p->y > 2) {
        normal.y = -1;
        collisionForce.y += kCollision * (2 - p->y);
    }

    // Check Z boundaries
    if (p->z < -2) {
        normal.z = 1;
        collisionForce.z += kCollision * (-2 - p->z);
    } else if (p->z > 2) {
        normal.z = -1;
        collisionForce.z += kCollision * (2 - p->z);
    }
    // EXTRA CREDIT Check collision with the inclined plane
    if (jello->incPlanePresent) {
        double distance = jello->a * p->x + jello->b * p->y + jello->c * p->z + jello->d;

        if (distance < 0) { // If below the plane
            double penetrationDepth = -distance;

            // Compute normal force pushing the point back
            struct point planeForce;
            planeForce.x = kCollision * penetrationDepth * jello->a;
            planeForce.y = kCollision * penetrationDepth * jello->b;
            planeForce.z = kCollision * penetrationDepth * jello->c;

            // Apply damping force in the opposite direction of velocity
            struct point velocityDamping;
            velocityDamping.x = -dCollision * v->x;
            velocityDamping.y = -dCollision * v->y;
            velocityDamping.z = -dCollision * v->z;

            // Add both forces to total collision force
            pSUM(collisionForce, planeForce, collisionForce);
            pSUM(collisionForce, velocityDamping, collisionForce);
        }
    }

    // Apply damping only if there's a collision
    if (normal.x != 0 || normal.y != 0 || normal.z != 0) {
        double dampingMagnitude = -dCollision * (v->x * normal.x + v->y * normal.y + v->z * normal.z);
        pMULTIPLY(normal, dampingMagnitude, dampingForce);
        pSUM(*totalForce, dampingForce, *totalForce);
    }

    // Add collision force to total force
    pSUM(*totalForce, collisionForce, *totalForce);
}


#include <algorithm> // For std::max and std::min

// Helper function to get clamped grid indices
void getGridIndices(const struct world *jello, double x, double y, double z, int &ix, int &iy, int &iz, double &fx, double &fy, double &fz) {
    int res = jello->resolution;
    double step = 4.0 / (res - 1); // Step size for force field grid

    // Convert world coordinates to force field grid indices
    ix = (x + 2) / step;
    iy = (y + 2) / step;
    iz = (z + 2) / step;

    // Clamp indices to ensure they stay within bounds
    ix = std::max(0, std::min(ix, res - 2));
    iy = std::max(0, std::min(iy, res - 2));
    iz = std::max(0, std::min(iz, res - 2));

    // Compute interpolation weights
    fx = ((x + 2) - ix * step) / step;
    fy = ((y + 2) - iy * step) / step;
    fz = ((z + 2) - iz * step) / step;
}

// Helper function to fetch the force field values at given indices
void getSurroundingForces(const struct world *jello, int ix, int iy, int iz, int res, struct point forces[8]) {
    forces[0] = jello->forceField[ix * res * res + iy * res + iz];         // f000
    forces[1] = jello->forceField[ix * res * res + iy * res + (iz + 1)];   // f001
    forces[2] = jello->forceField[ix * res * res + (iy + 1) * res + iz];   // f010
    forces[3] = jello->forceField[ix * res * res + (iy + 1) * res + (iz + 1)]; // f011
    forces[4] = jello->forceField[(ix + 1) * res * res + iy * res + iz];   // f100
    forces[5] = jello->forceField[(ix + 1) * res * res + iy * res + (iz + 1)]; // f101
    forces[6] = jello->forceField[(ix + 1) * res * res + (iy + 1) * res + iz]; // f110
    forces[7] = jello->forceField[(ix + 1) * res * res + (iy + 1) * res + (iz + 1)]; // f111
}

// Helper function to perform trilinear interpolation
struct point interpolateForce(const struct point forces[8], double fx, double fy, double fz) {
    struct point interpForce;

    interpForce.x = (1 - fx) * (1 - fy) * (1 - fz) * forces[0].x +
                    (1 - fx) * (1 - fy) * fz * forces[1].x +
                    (1 - fx) * fy * (1 - fz) * forces[2].x +
                    (1 - fx) * fy * fz * forces[3].x +
                    fx * (1 - fy) * (1 - fz) * forces[4].x +
                    fx * (1 - fy) * fz * forces[5].x +
                    fx * fy * (1 - fz) * forces[6].x +
                    fx * fy * fz * forces[7].x;

    interpForce.y = (1 - fx) * (1 - fy) * (1 - fz) * forces[0].y +
                    (1 - fx) * (1 - fy) * fz * forces[1].y +
                    (1 - fx) * fy * (1 - fz) * forces[2].y +
                    (1 - fx) * fy * fz * forces[3].y +
                    fx * (1 - fy) * (1 - fz) * forces[4].y +
                    fx * (1 - fy) * fz * forces[5].y +
                    fx * fy * (1 - fz) * forces[6].y +
                    fx * fy * fz * forces[7].y;

    interpForce.z = (1 - fx) * (1 - fy) * (1 - fz) * forces[0].z +
                    (1 - fx) * (1 - fy) * fz * forces[1].z +
                    (1 - fx) * fy * (1 - fz) * forces[2].z +
                    (1 - fx) * fy * fz * forces[3].z +
                    fx * (1 - fy) * (1 - fz) * forces[4].z +
                    fx * (1 - fy) * fz * forces[5].z +
                    fx * fy * (1 - fz) * forces[6].z +
                    fx * fy * fz * forces[7].z;

    return interpForce;
}

// Main function to apply external force using interpolation
void applyExternalForce(struct world *jello, int i, int j, int k, struct point *totalForce) {
    if (jello->resolution == 0) return; // No force field

    // Get current mass point position
    double x = jello->p[i][j][k].x;
    double y = jello->p[i][j][k].y;
    double z = jello->p[i][j][k].z;

    int ix, iy, iz;
    double fx, fy, fz;

    // Get clamped grid indices and interpolation weights
    getGridIndices(jello, x, y, z, ix, iy, iz, fx, fy, fz);

    // Fetch surrounding force values
    struct point forces[8];
    getSurroundingForces(jello, ix, iy, iz, jello->resolution, forces);

    // Interpolate force field value at the current point
    struct point interpForce = interpolateForce(forces, fx, fy, fz);

    // Add the interpolated force to the total force
    pSUM(*totalForce, interpForce, *totalForce);
}


void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /* for you to implement ... */
  int i, j, k;
      // Initialize acceleration array to zero
    for (i = 0; i < 8; i++)
        for (j = 0; j < 8; j++)
            for (k = 0; k < 8; k++)
                pMAKE(0, 0, 0, a[i][j][k]);
    for (i = 0; i < 8; i++)
        for (j = 0; j < 8; j++)
            for (k = 0; k < 8; k++) {
                struct point totalForce;
                pMAKE(0, 0, 0, totalForce);

                // Compute forces from structural, shear, and bend springs
                applySpringForces(jello, i, j, k, &totalForce);

                applyDampingForces(jello, i, j, k, &totalForce);

                // Apply collision forces
                applyCollisionForces(jello, i, j, k, &totalForce);

                applyExternalForce(jello, i, j, k, &totalForce);
                // Compute acceleration: a = F / m

                // Apply Gravity: F = m * g  -->  a = g
                //totalForce.z += jello->mass * (-9.81);
                pMULTIPLY(totalForce, 1.0 / jello->mass, a[i][j][k]);
            }
    

}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
