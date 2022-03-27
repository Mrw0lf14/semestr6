/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define GridSize 8

int** GridInit(int gen);
void GridShow(int** grid);
void GridCopy(int*** togrid, int** fromgrid);
void GridNext(int*** togrid, int** fromgrid);
int CellCheck(int** grid, int x, int y);

int main()
{
    int** grid1 = NULL;
    int** grid2 = NULL;
    grid1 = GridInit(1);
    grid2 = GridInit(0);
    GridCopy(&grid2, grid1);
    while(1){
        GridShow(grid1);
        GridNext(&grid1, grid2);
        GridCopy(&grid2, grid1);
        GridShow(grid1);
        getchar();
    }
    return 0;
}

int** GridInit(int gen){
    srand(1);
    int** grid = NULL;
    grid = (int**)malloc(sizeof(int*)*GridSize);
    for (int i = 0; i<GridSize; i++){
        grid[i] = (int*)malloc(sizeof(int)*GridSize);
        for(int j = 0; j < GridSize; j++){
            if(gen!=0)
                grid[i][j] = rand()%2;
            else
                grid[i][j] = 0;
        }
    }
    return grid;
}
void GridShow(int** grid){
    for(int i = 0; i < GridSize; i++){
        for(int j = 0; j < GridSize; j++){
            printf("%d", grid[i][j]);
            printf(" ");
        }
        printf("\n");
    }
    printf("\n");
}

void GridCopy(int*** togrid, int** fromgrid){
    int** temp = *togrid;
    for(int i = 0; i < GridSize; i++){
        for(int j = 0; j < GridSize; j++){
            temp[i][j] = fromgrid[i][j];
        }
    }
}
int CellCheck(int** grid, int x, int y){
    int count = 0;
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            if(i!=0 && j!=0)
            if((x+i>=0) && (x+i<GridSize)
            && (y+j>=0) && (y+j<GridSize)){
                count += grid[x+i][y+i];
            }
        }
    }
    return count;
}
void GridNext(int*** togrid, int** fromgrid){
    int** temp = *togrid;
    for(int i = 0; i < GridSize; i++){
        for(int j = 0; j < GridSize; j++){
            if(fromgrid[i][j] == 0){
                if(CellCheck(fromgrid, i, j) == 3){
                    temp[i][j] = 1;
                }
            }
            else{
                if((CellCheck(fromgrid, i, j) == 3) || (CellCheck(fromgrid, i, j) == 2)){
                    temp[i][j] = 1;
                }
                else
                    temp[i][j] = 0;
            }
            temp[i][j] = CellCheck(fromgrid, i, j);
        }
    }
}
