
#ifndef BTREE_H
#define BTREE_H

/*
 * Include some necessary headers.
 */

#include<iostream>
#include "btreenode.h"
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>


using namespace argos;

class Btree: public CCI_Controller {
    
public:
    // Constructor (Initializes tree as empty)
    BTree()
    {  root = NULL; }
    
    // function to traverse the tree
    void traverse()
    {  if (root != NULL) root->traverse(); }
    
    // The main function that inserts a new key in this B-Tree
    void insert(CFootBotEntity _robot);
    
private:
    
    BTreeNode *root; // Pointer to root node
    
};

#endif
