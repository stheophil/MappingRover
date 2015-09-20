
#ifdef __cplusplus
extern "C"
#endif
int find_path(int nStartX, int nStartY,
               int nDirection,
               int nGoalX, int nGoalY,
               unsigned char const* pbImage,
               unsigned int cbBytesPerRow,
               int nExtent,
               void (^foreachPoint)(int nX, int nY));
