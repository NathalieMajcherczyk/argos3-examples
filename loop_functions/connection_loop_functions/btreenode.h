//www.geeksforgeeks.org/b-tree-set-1-insert-2

#ifndef BTREENODE_H
#define BTREENODE_H

/*
 * Include some necessary headers.
 */

#include<iostream>
#include "btree.h"
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class BTreeNode: public CCI_Controller {
    
public:

    BTreeNode(bool leaf);   // Constructor
    
    // A utility function to insert a new robot in the subtree rooted with
    // this node. The assumption is, the node must be non-full when this
    // function is called
    void insert(CFootBotEntity _robot);
    
    // A function to traverse all nodes in a subtree rooted with this node
    void traverse();
    
    // A function to search distance in subtree rooted with this node.
    // BTreeNode *search(Real distance);

private:
    
    CFootBotEntity *m_Robots; // Array of robots at one level
    std::vector<BTreeNode> *m_Children; // Vector of child pointers
    int m_NumberRobots; // Number of robots at current level :needed ?
    bool m_leaf; // True when node is leaf : needed ?
    
friend class BTree;
};

#endif
