/**
 * @file kdtree.h
 * @author Aaron Brown 
 *
 * @brief 
 * @version 0.1
 * @date 2020-04-06
 * 
 *
 * 
 */

#ifndef KD_TREE_H
#define KD_TREE_H

#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id, int dims)
	{
      	if(*node == NULL)
      	{
        	*node = new Node(point, id);
		}
      	else
      	{
			uint cd = depth % dims;

			if (point[cd] < (*node)->point[cd])
			{
				insertHelper( &((*node)->left) ,depth+1,point,id,dims);
			}
      		else
      		{
        		insertHelper( &((*node)->right) ,depth+1,point,id,dims);
      		}
		}
	}

	void insert(std::vector<float> point, int id, int dims)
	{
		insertHelper(&root,0,point,id,dims);
 	}

	void searchHelper2D(std::vector<float> target, Node* node,
					   int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node!=NULL)
		{
			if (( node->point[0]>=(target[0] - distanceTol)
		   	  &&  node->point[0]<=(target[0] + distanceTol))
			  && (node->point[1]>=(target[1] - distanceTol)
			  &&  node->point[1]<=(target[1] + distanceTol)))
			{
				float distance = sqrt( (node->point[0]-target[0])*
									   (node->point[0]-target[0])+
									   (node->point[1]-target[1])*
									   (node->point[1]-target[1]));

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}

				if ((target[depth%2] - distanceTol) < node->point[depth%2])
				{
					searchHelper2D(target, node->left, depth+1, distanceTol, ids);
				}

				if ((target[depth%2] + distanceTol) > node->point[depth%2])
				{
					searchHelper2D(target, node->right, depth+1, distanceTol, ids);
					}
			}

		}
	}

	void searchHelper3D(std::vector<float> target, Node* node,
					   int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node!=NULL)
		{
			if (( node->point[0]>=(target[0] - distanceTol)
		   	  &&  node->point[0]<=(target[0] + distanceTol))
			  && (node->point[1]>=(target[1] - distanceTol)
			  &&  node->point[1]<=(target[1] + distanceTol))
			  && (node->point[2]>=(target[2] - distanceTol)
			  &&  node->point[2]<=(target[2] + distanceTol))
			  )
			{
				float distance = sqrt( (node->point[0]-target[0])*
									   (node->point[0]-target[0])+
									   (node->point[1]-target[1])*
									   (node->point[1]-target[1])+
									   (node->point[2]-target[2])*
									   (node->point[2]-target[2])
									   );

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}

				if ((target[depth%3] - distanceTol) < node->point[depth%3])
				{
					searchHelper3D(target, node->left, depth+1, distanceTol, ids);
				}

				if ((target[depth%3] + distanceTol) > node->point[depth%3])
				{
					searchHelper3D(target, node->right, depth+1, distanceTol, ids);
				}
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper3D(target, root, 0, distanceTol, ids);
		return ids;
	}
};

#endif /* KD_TREE_H */