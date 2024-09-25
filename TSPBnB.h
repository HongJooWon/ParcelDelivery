/*


#include <bits/stdc++.h>
using namespace std;

// number of total nodes
#define N 5
#define INF INT_MAX

class Node
{
public:
    vector<pair<int, int>> path;
    int matrix_reduced[N][N];
    int cost;
    int vertex;
    int level;
};

Node* newNode(int matrix_parent[N][N], vector<pair<int, int>> const &path,int level, int i, int j)
{
    Node* node = new Node;
    node->path = path;
    if (level != 0)
        node->path.push_back(make_pair(i, j));
    memcpy(node->matrix_reduced, matrix_parent,
        sizeof node->matrix_reduced);
    for (int k = 0; level != 0 && k < N; k++)
    {
        node->matrix_reduced[i][k] = INF;
        node->matrix_reduced[k][j] = INF;
    }

    node->matrix_reduced[j][0] = INF;
    node->level = level;
    node->vertex = j;
    return node;
}

void reduce_row(int matrix_reduced[N][N], int row[N]);
void reduce_column(int matrix_reduced[N][N], int col[N]);
int cost_calculation(int matrix_reduced[N][N]);
void printPath(vector<pair<int, int>> const &list);

class comp {
public:
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return lhs->cost > rhs->cost;
    }
};

int solve(int adjacensyMatrix[N][N]);



*/
