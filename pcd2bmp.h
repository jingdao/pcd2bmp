#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
    unsigned int bmpSize;
    int width;
    int height;
    unsigned short bitsPerPixel;
    unsigned int imageSize;
    char* data;
} Bitmap;

typedef enum PCD_field_type_label{
    X,
    Y,
    Z,
    RGB,
    NORMAL,
    MOMENT_INVARIANT
}PCD_field_type;

typedef enum PCD_data_storage_label {
    ASCII,
    BINARY
}PCD_data_storage;

typedef enum PCD_file_format_label {
    VERSION,
    FIELDS,
    SIZE,
    TYPE,
    COUNT,
    WIDTH,
    HEIGHT,
    VIEWPOINT,
    POINTS,
    DATA,
    POINT_CLOUD
}PCD_file_format;

typedef struct {
    PCD_field_type type;
    int size;
    int count;
    char data_type;
} PCD_Field;

typedef struct {
    float r;
    float i;
    float j;
    float k;
} Quaternion;

typedef struct {
    Quaternion position;
    Quaternion rotation;
    float focal_length;
} Camera;

typedef struct {
    int height;
    int width;
    float viewpoint[7];
    Camera camera;
    int numFields;
    PCD_Field* fields;
    char* data;
    float* float_data;
    int numPoints;
    PCD_data_storage data_storage;
} PCD;

PCD* NewPCDFromFile(char* fileName);
PCD* NewPCD4FromFile(char* fileName);
void DeletePCD(PCD* pcd);
Bitmap* NewBitmap(unsigned int width, unsigned int height, unsigned short bitsPerPixel);
void DeleteBitmap(Bitmap* bmp);
void SaveBitmap(Bitmap* bmp, char* fileName);
int Show3DProjection(Bitmap* bmp, PCD* pcd, int pointSize, int zoomToFit);
void pcd2bmp(char* src, char* dest, int pointSize);
