/**
  *    DaLI: Deformation and Light Invariant Descriptor
  *    Edgar Simo-Serra, Carme Torras, Francesc Moreno-Noguer
  *    International Journal of Computer Vision (IJCV), 2015
  *
  * Copyright (C) <2011-2015>  <Francesc Moreno-Noguer, Edgar Simo-Serra>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the version 3 of the GNU General Public License
  * as published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  * General Public License for more details.      
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  * Edgar Simo-Serra, Institut de Robotica i Informatica Industrial (CSIC/UPC)
  * esimo@iri.upc.edu, http://www-iri.upc.es/people/esimo/
 **/


#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>


#include "dali_internal.h"


/** 
 * Meshing type details.
 */
typedef enum BlockMesh_e {
   /*  +---+
    *  |   |
    *  |   |
    *  |   |
    *  +---+  */
   BLOCKMESH_NIL,
   BLOCKMESH_SKIP,
   /*  +---+
    *  |  /|
    *  | / |
    *  |/  |
    *  +---+  */
   BLOCKMESH_DIAG_UR,
   BLOCKMESH_DIAG_DR,
   /*  +---+
    *  |\ /|
    *  | o |
    *  |/ \|
    *  +---+  */
   BLOCKMESH_MIDDLE,
   /*  +---+
    *  |\ /|
    *  | o-+
    *  |/|\|
    *  +-+-+  */
   BLOCKMESH_CHILD_BR,
   /*  +---+
    *  |\ /|
    *  +-o-+
    *  |/|\|
    *  +-+-+  */
   BLOCKMESH_CHILD_B,
   BLOCKMESH_CHILD_R,
} BlockMesh_t;


/**
 * Helper struct to create the block tree for meshing.
 */
typedef struct BlockTree_s {
   /* Meshing type. */
   BlockMesh_t mesh;

   /* Tree details. */
   struct BlockTree_s *parent;
   struct BlockTree_s *children; /* Always 4. */

   /* Position details. */
   int pos; /* Relative position. */
   double u,v;
   double w,h;
} BlockTree_t;

static void btree_free( BlockTree_t *btree )
{
   int i;
   /* Free children. */
   if (btree->children != NULL) {
      for (i=0; i<4; i++)
         btree_free( &btree->children[i] );
      free( btree->children );
   }
}
/**
 * @brief Initializes a specific btree node.
 *
 *    @param btree Node to initialize.
 *    @param parent Parent of the node.
 *    @param u Bottom-left corner U coordinate.
 *    @param v Bottom-left corner V coordinate.
 *    @param w Width of the node.
 *    @param h Height of the node.
 *    @param pos Position of the node (1=top-left, 2=top-right.
 *          3=bottom-left, 4=bottom-right)
 */
static void btree_set( BlockTree_t *btree, BlockTree_t *parent,
      double u, double v, double w, double h, int pos )
{
   btree->mesh = BLOCKMESH_NIL;
   /* Tree stuff. */
   btree->parent     = parent;
   btree->children   = NULL;
   /* Details. */
   btree->u = u;
   btree->v = v;
   btree->w = w;
   btree->h = h;
   btree->pos = pos;
}
/**
 * @brief Splits a node into four children nodes.
 * 
 *    @param btree Node to split.
 */
static void btree_split( BlockTree_t *btree )
{
   double u,v, w,h;
   /* For ease. */
   u = btree->u;
   v = btree->v;
   w = 0.5*btree->w;
   h = 0.5*btree->h;
   /* Allocate. */
   btree->children = malloc( sizeof(BlockTree_t)*4 );
   assert( btree->children != NULL );
   /* Split the four blocks. */
   btree_set( &btree->children[0], btree, u,   v+h, w, h, 1 );
   btree_set( &btree->children[1], btree, u+w, v+h, w, h, 2 );
   btree_set( &btree->children[2], btree, u,   v,   w, h, 3 );
   btree_set( &btree->children[3], btree, u+w, v,   w, h, 4 );
}
/**
 * @brief Evaluates a gaussian function at offset u,v with parameters K, sigma.
 *
 *    @param u U offset.
 *    @param v V offset.
 *    @param K Gaussian gain.
 *    @param sigma Standard deviation of the Gaussian.
 *    @return The value at the point.
 */
static double gauss_eval( double u, double v, double K, double sigma )
{
   return K*exp( -(pow(u,2) + pow(v,2)) / sigma );
}
/**
 * @brief Recursively splits children to approximate a Gaussian function.
 *
 * @note This function only does it with blocks and does not perform any
 * sort of meshing.
 *
 *    @param btree Node to check.
 *    @param K Gain of the Gaussian.
 *    @param sigma Deviation of the Gaussian.
 */
static void mesh_recursiveSplit( BlockTree_t *btree,
      double K, double sigma )
{
   int i;
   btree_split( btree );
   for (i=0; i<4; i++) {
      BlockTree_t *child;
      double u,v, w,h;
      double dens, eval;

      /* Get child properties. */
      child = &btree->children[i];
      w     = child->w;
      h     = child->h;
      u     = child->u + 0.5*w;
      v     = child->v + 0.5*h;

      /* Make sure density is ok. */
      dens  = 1./(w*h);
      eval  = gauss_eval( u, v, K, sigma );
      if (4.*dens > eval)
         continue;

      /* Split the child. */
      mesh_recursiveSplit( child, K, sigma );
   }
}
/**
 * @brief Sets a Face.
 *
 *    @param face Face to set.
 *    @param a First node.
 *    @param b Second node.
 *    @param c Third node.
 */
static void face_set( int *face, int a, int b, int c )
{
   face[0] = a;
   face[1] = b;
   face[2] = c;
}
/**
 * @brief Sets a face if it's in range (checks center of triangle).
 *
 *    @param nfaces Number of faces in the array.
 *    @param face Initial face of the array.
 *    @param a First node.
 *    @param b Second node.
 *    @param c Third node.
 *    @param nodes Nodes in the array.
 *    @param sz2 The square of the radius to check.
 *    @return The ID of the face if added or -1 if not added.
 */
static int face_setRange( int *nfaces, int *face, int a, int b, int c,
      double *nodes, double sz2 )
{
   double u,v, dist;
   u = (nodes[2*a+0]+nodes[2*b+0]+nodes[2*c+0])/3.;
   v = (nodes[2*a+1]+nodes[2*b+1]+nodes[2*c+1])/3.;

   dist = POW2(fabs(u)) + POW2(fabs(v));
   if (dist > sz2)
      return -1;

   int f = *nfaces;
   face_set( &face[3*f], a, b, c );
   (*nfaces) = f+1;
   return f;
}
/**
 * @brief Checks to see if a node already exists or adds it if it does not.
 *
 *    @param nnodes Number of nodes in the array.
 *    @param nodes Node arary.
 *    @param u U coordinate of the new node.
 *    @param v V coordinate of the new node.
 *    @return ID of the existing or newly created node.
 */
static int node_checkAdd( int *nnodes, double *nodes, double u, double v )
{
   int i;
   /* Try to find it first. */
   for (i=0; i<(*nnodes); i++) {
      double d = POW2(nodes[i*2+0]-u) + POW2(nodes[i*2+1]-v);
      if (d < 1e-5)
         return i;
   }

   /* Add to the list. */
   i = *nnodes;
   nodes[i*2+0] = u;
   nodes[i*2+1] = v;
   *nnodes = i+1;
   return i;
}
/**
 * @brief Recurcsively performs the meshing on an existing tree that is already
 *        set up with mesh values.
 *
 *    @param btree Node being checked.
 *    @param nfaces Number of faces in the array.
 *    @param faces Face arary.
 *    @param nnodes Number of the nodes in the array.
 *    @param nodes Node array.
 */
static void mesh_recursiveFaces( const BlockTree_t *btree,
      int *nfaces, int *faces, int *nnodes, double *nodes )
{
   int i;
   int tl,tr,bl,br, e1,e2,e3;
   double u,v, w,h, uc,vc;
   int inside;
   double sz2 = 1.;

   /* Dimensions. */
   u     = btree->u;
   v     = btree->v;
   w     = btree->w;
   h     = btree->h;
   uc    = u + 0.5*w;
   vc    = v + 0.5*h;

   /* Add nodes. */
   bl = node_checkAdd( nnodes, nodes, u,   v   );
   br = node_checkAdd( nnodes, nodes, u+w, v   );
   tl = node_checkAdd( nnodes, nodes, u,   v+h );
   tr = node_checkAdd( nnodes, nodes, u+w, v+h );

   switch (btree->mesh) {
      case BLOCKMESH_NIL:
         if (btree->children != NULL) {
            for (i=0; i<4; i++)
               mesh_recursiveFaces( &btree->children[i],
                     nfaces, faces, nnodes, nodes );
         }
         break;
      case BLOCKMESH_SKIP:
         break;

      case BLOCKMESH_DIAG_UR:
         face_setRange( nfaces, faces, bl, tl, tr, nodes, sz2 );
         face_setRange( nfaces, faces, bl, br, tr, nodes, sz2 );
         break;
      case BLOCKMESH_DIAG_DR:
         face_setRange( nfaces, faces, tl, tr, br, nodes, sz2 );
         face_setRange( nfaces, faces, tl, bl, br, nodes, sz2 );
         break;

      case BLOCKMESH_MIDDLE:
         inside = node_checkAdd( nnodes, nodes, uc, vc );
         face_setRange( nfaces, faces, tl, tr, inside, nodes, sz2 );
         face_setRange( nfaces, faces, tr, br, inside, nodes, sz2 );
         face_setRange( nfaces, faces, br, bl, inside, nodes, sz2 );
         face_setRange( nfaces, faces, bl, tl, inside, nodes, sz2 );
         break;

      case BLOCKMESH_CHILD_BR:
         /*  +---+
          *  |\ /|
          *  | o-+
          *  |/|\|
          *  +-+-+  */
         e1     = node_checkAdd( nnodes, nodes, u+w, vc );
         e2     = node_checkAdd( nnodes, nodes, uc, v  );
         inside = node_checkAdd( nnodes, nodes, uc, vc );
         face_setRange( nfaces, faces, tl, tr, inside, nodes, sz2 ); /* top tri */
         face_setRange( nfaces, faces, tl, bl, inside, nodes, sz2 ); /* left tri */
         face_setRange( nfaces, faces, tr, e1, inside, nodes, sz2 ); /* right-top tri */
         face_setRange( nfaces, faces, e2, bl, inside, nodes, sz2 ); /* bottom-left tri */
         if (btree->children != NULL) {
            mesh_recursiveFaces( &btree->children[3],
                  nfaces, faces, nnodes, nodes );
         }
         break;
      case BLOCKMESH_CHILD_B:
         e1     = node_checkAdd( nnodes, nodes, u+w, vc );
         e2     = node_checkAdd( nnodes, nodes, uc,  v  );
         e3     = node_checkAdd( nnodes, nodes, u,   vc );
         inside = node_checkAdd( nnodes, nodes, uc,  vc );
         face_setRange( nfaces, faces, tl, tr, inside, nodes, sz2 ); /* top tri */
         face_setRange( nfaces, faces, tr, e1, inside, nodes, sz2 ); /* right-top tri */
         face_setRange( nfaces, faces, tl, e3, inside, nodes, sz2 ); /* left-top tri */
         if (btree->children != NULL) {
            mesh_recursiveFaces( &btree->children[2],
                  nfaces, faces, nnodes, nodes );
            mesh_recursiveFaces( &btree->children[3],
                  nfaces, faces, nnodes, nodes );
         }
         break;
      case BLOCKMESH_CHILD_R:
         e1     = node_checkAdd( nnodes, nodes, u+w, vc  );
         e2     = node_checkAdd( nnodes, nodes, uc,  v   );
         e3     = node_checkAdd( nnodes, nodes, uc,  v+h );
         inside = node_checkAdd( nnodes, nodes, uc,  vc  );
         face_setRange( nfaces, faces, tl, bl, inside, nodes, sz2 ); /* left tri */
         face_setRange( nfaces, faces, e2, bl, inside, nodes, sz2 ); /* bottom-left tri */
         face_setRange( nfaces, faces, tl, e3, inside, nodes, sz2 ); /* top-left tri */
         if (btree->children != NULL) {
            mesh_recursiveFaces( &btree->children[1],
                  nfaces, faces, nnodes, nodes );
            mesh_recursiveFaces( &btree->children[3],
                  nfaces, faces, nnodes, nodes );
         }
         break;

      default:
         break;
   }
}
/**
 * @brief Fixes up the meshing on a node given it's adjacent neighbours.
 *
 *    @param btree Node to be fixed up.
 *    @param n1 Neighbour adjacent to the right.
 *    @param n2 Neighbour adjacent to the south.
 *    @param K Gaussian gain.
 *    @param sigma Gaussian standard deviation.
 *    @return Roughly the number of faces added.
 */
static int btree_fixup1( BlockTree_t *btree,
      BlockTree_t *n1, BlockTree_t *n2,
      double K, double sigma )
{
   int r = 0;
   int h1, h2;

   /* Stuff. */
   h1 = (n1!=NULL) && (n1->children != NULL) &&
         ((n1->mesh == BLOCKMESH_NIL) );
   h2 = (n2!=NULL) && (n2->children != NULL) &&
         ((n2->mesh == BLOCKMESH_NIL) );

   /* Neither have children. */
   if (!h1 && !h2) {
      if ((n1 != NULL) && (n2 != NULL)) {
         int i;
         if ((n1->mesh == BLOCKMESH_CHILD_B) &&
             (n2->mesh == BLOCKMESH_CHILD_R)) {
            btree->mesh = BLOCKMESH_CHILD_BR;
            btree_split( btree );
            btree->children[0].mesh = BLOCKMESH_SKIP;
            btree->children[1].mesh = BLOCKMESH_SKIP;
            r += btree_fixup1( &btree->children[3],
                  &n1->children[2],
                  &n2->children[1], K, sigma );
            return r+4;
         }
         else if ((n1->mesh == BLOCKMESH_CHILD_R) &&
                  (n2->mesh == BLOCKMESH_CHILD_R)) {
            n1->mesh = BLOCKMESH_NIL;
            for (i=0; i<4; i++)
               if ((n1->children[i].mesh == BLOCKMESH_SKIP) ||
                   (n1->children[i].mesh == BLOCKMESH_NIL)) {
                  n1->children[i].mesh = BLOCKMESH_DIAG_UR;
                  r += 2;
               }
            btree->mesh = BLOCKMESH_CHILD_BR;
            btree_split( btree );
            btree->children[0].mesh = BLOCKMESH_SKIP;
            btree->children[1].mesh = BLOCKMESH_SKIP;
            r += btree_fixup1( &btree->children[3],
                  &n1->children[2],
                  &n2->children[1], K, sigma );
            return r+4;
         }
         else if ((n1->mesh == BLOCKMESH_CHILD_B) &&
                  (n2->mesh == BLOCKMESH_CHILD_B)) {
            n2->mesh = BLOCKMESH_NIL;
            for (i=0; i<4; i++)
               if ((n2->children[i].mesh == BLOCKMESH_SKIP) ||
                   (n2->children[i].mesh == BLOCKMESH_NIL)) {
                  n2->children[i].mesh = BLOCKMESH_DIAG_UR;
                  r += 2;
               }
            btree->mesh = BLOCKMESH_CHILD_BR;
            btree_split( btree );
            btree->children[0].mesh = BLOCKMESH_SKIP;
            btree->children[1].mesh = BLOCKMESH_SKIP;
            r += btree_fixup1( &btree->children[3],
                  &n1->children[2],
                  &n2->children[1], K, sigma );
            return r+4;
         }
         else if ((n1->mesh == BLOCKMESH_CHILD_BR) &&
                  (n2->mesh == BLOCKMESH_CHILD_R)) {
            n1->mesh = BLOCKMESH_CHILD_B;
            n1->children[2].mesh = BLOCKMESH_DIAG_UR;
            btree->mesh = BLOCKMESH_CHILD_BR;
            btree_split( btree );
            btree->children[0].mesh = BLOCKMESH_SKIP;
            btree->children[1].mesh = BLOCKMESH_SKIP;
            r += btree_fixup1( &btree->children[3],
                  &n1->children[2],
                  &n2->children[1], K, sigma );
            return r+3;
         }
         else if ((n1->mesh == BLOCKMESH_CHILD_B) &&
                  (n2->mesh == BLOCKMESH_CHILD_BR)) {
            n2->mesh = BLOCKMESH_CHILD_R;
            n2->children[1].mesh = BLOCKMESH_DIAG_UR;
            btree->mesh = BLOCKMESH_CHILD_BR;
            btree_split( btree );
            btree->children[0].mesh = BLOCKMESH_SKIP;
            btree->children[1].mesh = BLOCKMESH_SKIP;
            r += btree_fixup1( &btree->children[3],
                  &n1->children[2],
                  &n2->children[1], K, sigma );
            return r+3;
         }
      }

      double u,v, w,h, uc,vc;
      double dens, eval;

      /* Dimensions. */
      u     = btree->u;
      v     = btree->v;
      w     = btree->w;
      h     = btree->h;
      uc    = u + 0.5*w;
      vc    = v + 0.5*h;

      dens  = 1./(w*h);
      eval  = gauss_eval( uc, vc, K, sigma );

      if (2.*dens > eval) {
         btree->mesh = BLOCKMESH_DIAG_UR;
         return 2;
      }
      else {
         btree->mesh = BLOCKMESH_MIDDLE;
         return 4;
      }
   }
   /* Both have children. */
   else if (h1 && h2) {
      btree->mesh = BLOCKMESH_CHILD_BR;
      btree_split( btree );
      btree->children[0].mesh = BLOCKMESH_SKIP;
      btree->children[1].mesh = BLOCKMESH_SKIP;
      r += btree_fixup1( &btree->children[3],
            &n1->children[2],
            &n2->children[1], K, sigma );
      return r+4;
   }
   /* Only east has children. */
   else if (h1 && !h2) {
      if (n2 != NULL) {
         if (n2->mesh == BLOCKMESH_CHILD_BR) {
            n2->mesh = BLOCKMESH_CHILD_R;
            n2->children[1].mesh = BLOCKMESH_DIAG_UR;
            r += 1;
         }
         else if (n2->mesh == BLOCKMESH_CHILD_B) {
            n2->mesh = BLOCKMESH_NIL;
            n2->children[0].mesh = BLOCKMESH_DIAG_UR;
            n2->children[1].mesh = BLOCKMESH_DIAG_UR;
            r += 4;
         }
      }
      if (n1->children[0].children == NULL) {
         btree->mesh = BLOCKMESH_CHILD_BR;
         btree_split( btree );
         btree->children[0].mesh = BLOCKMESH_SKIP;
         btree->children[1].mesh = BLOCKMESH_SKIP;
         r += btree_fixup1( &btree->children[3],
               &n1->children[2],
               NULL, K, sigma ) + 4;
      }
      else {
         btree->mesh = BLOCKMESH_CHILD_R;
         btree_split( btree );
         btree->children[0].mesh = BLOCKMESH_SKIP;
         btree->children[2].mesh = BLOCKMESH_SKIP;
         r += btree_fixup1( &btree->children[3],
               &n1->children[2],
               NULL, K, sigma ) + 3;
         r += btree_fixup1( &btree->children[1],
               &n1->children[0],
               &btree->children[3], K, sigma );
      }
      return r;
   }
   /* Only south has children. */
   else if (!h1 && h2) {
      if (n1 != NULL) {
         if (n1->mesh == BLOCKMESH_CHILD_BR) {
            n1->mesh = BLOCKMESH_CHILD_B;
            n1->children[2].mesh = BLOCKMESH_DIAG_UR;
            r += 1;
         }
         else if (n1->mesh == BLOCKMESH_CHILD_R) {
            n1->mesh = BLOCKMESH_NIL;
            n1->children[0].mesh = BLOCKMESH_DIAG_UR;
            n1->children[2].mesh = BLOCKMESH_DIAG_UR;
            r += 4;
         }
      }
      if (n2->children[0].children == NULL) {
         btree->mesh = BLOCKMESH_CHILD_BR;
         btree_split( btree );
         btree->children[0].mesh = BLOCKMESH_SKIP;
         btree->children[1].mesh = BLOCKMESH_SKIP;
         r += btree_fixup1( &btree->children[3],
               NULL,
               &n2->children[1], K, sigma ) + 4;
      }
      else {
         btree->mesh = BLOCKMESH_CHILD_B;
         btree_split( btree );
         btree->children[0].mesh = BLOCKMESH_SKIP;
         btree->children[1].mesh = BLOCKMESH_SKIP;
         r += btree_fixup1( &btree->children[3],
               NULL,
               &n2->children[1], K, sigma ) + 3;
         r += btree_fixup1( &btree->children[2],
               &btree->children[3],
               &n2->children[0], K, sigma );
      }
      return r;
   }
   else {
      assert( "what" == 0 );
   }
   return r;
}
/**
 * @brief Gets the neighbour to the east of a node.
 *
 *    @param btree Node to get neighbour east of.
 *    @return Neighbour east of the node or NULL if not found.
 */
static BlockTree_t* block_getNeighbourE( BlockTree_t *btree )
{
   BlockTree_t *parent, *gp;

   /* Top node. */
   parent = btree->parent;
   if (parent==NULL)
      return NULL;

   /* Easy cases. */
   if (btree->pos==1)
      return &parent->children[1];
   else if (btree->pos==3)
      return &parent->children[3];

   /* Get right parent. */
   gp = block_getNeighbourE( parent );
   if (gp == NULL)
      return NULL;

   if (btree->pos==2)
      return &gp->children[0];
   else if (btree->pos==4)
      return &gp->children[2];

   assert( "Invalid neighbour" == 0 );
}
/**
 * @brief Gets the neighbour to the south of a node.
 *
 *    @param btree Node to get neighbour south of.
 *    @return Neighbour south of the node or NULL if not found.
 */
static BlockTree_t* block_getNeighbourS( BlockTree_t *btree )
{
   BlockTree_t *parent, *gp;

   /* Top node. */
   parent = btree->parent;
   if (parent==NULL)
      return NULL;

   /* Easy cases. */
   if (btree->pos==1)
      return &parent->children[2];
   else if (btree->pos==2)
      return &parent->children[3];

   /* Get right parent. */
   gp = block_getNeighbourS( parent );
   if (gp == NULL)
      return NULL;

   if (btree->pos==3)
      return &gp->children[0];
   else if (btree->pos==4)
      return &gp->children[1];

   assert( "Invalid neighbour" == 0 );
}
/**
 * @brief Recursively sets the block type for a node used when meshing.
 *
 *    @param btree Node to set mesh type.
 *    @param K Gaussian gain.
 *    @param sigma Gaussian standard deviation.
 *    @return Roughly number of faces produced.
 */
static int mesh_blockType( BlockTree_t *btree,
      double K, double sigma )
{
   int i;

   /* Iterate until we find the leaves. */
   if (btree->children != NULL) {
      int r=0;
      /* Go down all children. */
      for (i=3; i>=0; i--) /* ORDER IS IMPORTANT */
         r += mesh_blockType( &btree->children[i], K, sigma );
      return r;
   }

   /* Complicated meshing. */
   BlockTree_t *n1, *n2;
   n1 = block_getNeighbourE( btree );
   n2 = block_getNeighbourS( btree );
   return btree_fixup1( btree, n1, n2, K, sigma );
}
/**
 * @brief Given the meshing for the upper-left quadrant, it replicates it to
 *        create a full mesh while discarding superfluous nodes.
 *
 *    @param nfaces Number of faces created. 
 *    @param faces Face output array.
 *    @param nnodes Number of nodes created.
 *    @param nodes Node output array.
 *    @param infaces Number of input faces.
 *    @param ifaces Input face array.
 *    @param innodes Number of input nodes.
 *    @param inodes Input node array.
 */
static void mesh_mirrorMerge( int *nfaces, int *faces, int *nnodes, double *nodes,
      const int infaces, const int *ifaces, const int innodes, const double *inodes )
{
   (void) innodes;
   int i, j, f;
   f = 0;
   for (i=0; i<infaces; i++) {

      int ntl[3], ntr[3], nbl[3], nbr[3];
      for (j=0; j<3; j++) {
         double u,v;

         u = inodes[ ifaces[3*i+j]*2+0 ];
         v = inodes[ ifaces[3*i+j]*2+1 ];

         ntl[j] = node_checkAdd( nnodes, nodes,  u,  v );
         ntr[j] = node_checkAdd( nnodes, nodes, -u,  v );
         nbl[j] = node_checkAdd( nnodes, nodes,  u, -v );
         nbr[j] = node_checkAdd( nnodes, nodes, -u, -v );
      }

      face_set( &faces[3*(f+0)], ntl[0], ntl[1], ntl[2] );
      face_set( &faces[3*(f+1)], ntr[0], ntr[1], ntr[2] );
      face_set( &faces[3*(f+2)], nbl[0], nbl[1], nbl[2] );
      face_set( &faces[3*(f+3)], nbr[0], nbr[1], nbr[2] );
      f += 4;
   }
   *nfaces = f;
}
/**
 * @brief Uses a tree structure to create a mesh where the density adapts a
 *        Gaussian function.
 *
 * @note This mesh is always circular and in ranges [-1,1].
 *
 *    @param mesh Mesh to set up.
 *    @param params Meshing parameters.
 */
void dali_meshComputeCircleGaussianTree( dali_mesh_t *mesh, const dali_params_t *params )
{
   double K = params->mesh_K;
   double sigma = params->mesh_sigma;

   /* Create the tree structure recursively. */
   BlockTree_t btree;
   btree_set( &btree, NULL, -1, 0, 1, 1, 0 );
   mesh_recursiveSplit( &btree, K, sigma );
   int r = mesh_blockType( &btree, K, sigma );

   /* Recursively perform the meshing. */
   int tnfaces = 0;
   int *tfaces = malloc( sizeof(int)*3 * 2*r );
   int tnnodes = 0;
   double *tnodes = malloc( sizeof(double)*2 * 6*r );
   assert( tfaces != NULL );
   assert( tnodes != NULL );
   mesh_recursiveFaces( &btree, &tnfaces, tfaces, &tnnodes, tnodes );

   /* Clean up. */
   btree_free( &btree );

   /* Generate the final mesh. */
   int nfaces = 0;
   int *faces = malloc( sizeof(int)*3 * 4*tnfaces );
   int nnodes = 0;
   double *nodes = malloc( sizeof(double)*2 * 4*tnnodes );
   assert( faces != NULL );
   assert( nodes != NULL );
   mesh_mirrorMerge( &nfaces,  faces, &nnodes,  nodes,
                     tnfaces, tfaces, tnnodes, tnodes );

   /* More clean up. */
   free( tfaces );
   free( tnodes );

   /* Set up mesh info. */
   free( mesh->F );
   free( mesh->V );
   mesh->F  = faces;
   mesh->nF = nfaces;
   mesh->oV = nodes;
   mesh->n  = nnodes;
   mesh->nr = nnodes;

   /* Allocate for the final mesh. */
   mesh->V  = malloc( sizeof(double)*3 * nnodes );
   assert( mesh->V != NULL );
}


/**
 * @brief Gets the value of a pixel in an image while providing mirroring if
 *        out of bounds.
 *
 *    @param u U coordinate.
 *    @param v V coordinate.
 *    @param im Image to use.
 *    @return The value at the point.
 */
static double im_value( int u, int v, const dali_img_t *im )
{
   /* Make sure u is in bounds. */
   if (u < 0)
      u = -u;
   else if (u >= im->w)
      u = 2*im->w - u - 1;
   /* Make sure v is in bounds. */
   if (v < 0)
      v = -v;
   else if (v >= im->h)
      v = 2*im->h - v - 1;
   /* Get point. */
   return im->data[ im->w*v + u ];
}
/**
 * @brief Performs bilinear interpolation on an image.
 *
 *    @param u U coordinate.
 *    @param v V coordinate.
 *    @param im IMage to read.
 *    @return Interpolated point of the image.
 */
static double im_interp( double u, double v, const dali_img_t *im )
{
   double u1,v1, u2,v2, u1o,v1o, u2o,v2o;
   double q11, q12, q22, q21;

   /* Get bound limits, we want to match pixels. */
   u1 = floor(u);
   v1 = floor(v);
   u2 = ceil(u);
   v2 = ceil(v);

   /* Get the position within the box. */
   u1o = u-u1;
   v1o = v-v1;
   u2o = u2-u;
   v2o = v2-v;

   /* Calculate the image at the points. */
   q11 = im_value( (int)u1, (int)v1, im );
   q12 = im_value( (int)u1, (int)v2, im );
   q22 = im_value( (int)u2, (int)v2, im );
   q21 = im_value( (int)u2, (int)v1, im );

   /* Interpolate bilinearly. */
   return 0.5*(q11*u2o*v2o + q21*u1o*v2o + q12*u2o*v1o + q22*u1o*v1o);
}
/**
 * @brief Performs bilinear interpolation while performing circular mirroring.
 */
static double im_interp_circ( double u, double v, const dali_img_t *im,
      double uc, double vc, double sz, double sz2 )
{
   double uo, vo, d;
   uo = u-uc;
   vo = v-vc;
   d  = POW2(uo)+POW2(vo);

   /* Inside circle, no problem. */
   if (d<sz2)
      return im_interp( u, v, im );

   /* Must mirror radially. */
   double a, l;
   a = atan2( vo, uo );
   l = -2.*(sqrt(d)-sz);

   return im_interp( u+l*cos(a), v+l*sin(a), im );
}

/**
 * @brief Computes 
 *
 *    @param desc Descriptor to fill properties of.
 *    @param mesh Meshing to use.
 *    @param im Input image.
 *    @params params Input parameters.
 */
void dali_meshComputeCircleGaussian( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params )
{
   int i;
   double sz, sz2;

   /* Load parameters. */
   sz    = params->Sz;
   sz2   = POW2(sz);

   /* Set stuff up. */
   desc->wlen  = params->wmax;
   desc->sz    = sz;
   desc->ulen  = 2.*sz+1.;
   desc->vlen  = 2.*sz+1.;
   desc->nodes = mesh->n;

   /* Perform scaling and interpolation. */
   mesh->len  = mesh->n;
   desc->mask = malloc( mesh->len * sizeof(int) );
   assert( desc->mask != NULL );
   for (i=0; i<mesh->n; i++) {
      double u, v;
      u = sz * mesh->oV[ i*2+0 ] + mesh->u;
      v = sz * mesh->oV[ i*2+1 ] + mesh->v;
      mesh->V[ i*3+0 ] = u;
      mesh->V[ i*3+1 ] = v;
      /*mesh->V[ i*3+2 ] = im_interp( u, v, im );*/
      mesh->V[ i*3+2 ] = im_interp_circ( u, v, im, mesh->u, mesh->v, sz, sz2 );
      /* Set mask shit up. */
      desc->mask[i] = 1;
   }
}


#if 0
int main( int argc, char *argv[] )
{
   dali_meshComputeCircleGaussianTree( 2500., 0.2);
   return 0;
}
#endif


