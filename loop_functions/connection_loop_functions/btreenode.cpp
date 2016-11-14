//www.geeksforgeeks.org/b-tree-set-1-insert-2

#include "btreenode.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

// Constructor for BTreeNode class
BTreeNode::BTreeNode(bool leaf)
{
    // Copy the given leaf property
    m_leaf = leaf;
    
    // Initialize the number of robots as 0
    m_NumberRobots = 0;
}

// Function to traverse all nodes in a subtree rooted with this node
void BTreeNode::traverse()
{
    // There are n keys and n+1 children, traverse through n robots
    // and first n children
    int i;
    for (i = 0; i < m_NumberRobots; i++)
    {
        // If this is not leaf, then before printing key[i],
        // traverse the subtree rooted with child C[i].
        if (m_leaf == false)
            m_Children[i]->traverse();
        LOG <<m_Robots[i].GetId()<<std::endl;
    }
    
    // Print the subtree rooted with last child
    if (leaf == false)
        m_Children[i]->traverse();
}

// A utility function to insert a new key in this node
// The assumption is, the node must be non-full when this
// function is called
void BTreeNode::insert(CFootBotEntity _robot)
{
    // Initialize index as index of rightmost element
    int i = n-1;
    
    // If this is a leaf node
    if (leaf == true)
    {
        ;
    }
    else // If this node is not leaf
    {
        // Find the child which is going to have the new key
        while (i >= 0)// //keys[i] > k)
            i--;
        C[i+1]->insert(_robot);
    }
}
