Index: image/image.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/image/image.cpp,v
retrieving revision 1.4
diff -u -r1.4 image.cpp
--- image/image.cpp	13 Jan 2003 19:41:03 -0000	1.4
+++ image/image.cpp	22 Jan 2005 09:18:25 -0000
@@ -58,9 +58,9 @@
 void MatrixImage<T>::drawLine(int i1, int j1, int i2, int j2, T color){
   int i,j ;
   double mx,b ;
-  if(i1<0 || j1<0 || i1>rows() || j1>=cols()  ){
+  if(i1<0 || j1<0 || i1>this->rows() || j1>=this->cols()  ){
 #ifdef USE_EXCEPTION
-    throw OutOfBound2D(i1,j1,0,rows()-1,0,cols()-1) ;
+    throw OutOfBound2D(i1,j1,0,this->rows()-1,0,this->cols()-1) ;
 #else
     Error error("MatrixImage<T>::drawLine") ;
     error << "Error in drawing line\n Invalid index ("<< i1 << ", " << j1 << ") to ( " << i2 << ", " << j2 << ") \n" ;
@@ -68,9 +68,9 @@
 #endif
     return ;
   }
-  if(i2 <0 || j2<0 || i2>rows() || j2>=cols() ){
+  if(i2 <0 || j2<0 || i2>this->rows() || j2>=this->cols() ){
 #ifdef USE_EXCEPTION
-    throw OutOfBound2D(i2,j2,0,rows()-1,0,cols()-1) ;
+    throw OutOfBound2D(i2,j2,0,this->rows()-1,0,this->cols()-1) ;
 #else
     Error error("MatrixImage<T>::drawLine") ;
     error << "Error in drawing line\n Invalid index ("<< i1 << ", " << j1 << ") to ( " << i2 << ", " << j2 << ") \n" ;
@@ -82,7 +82,7 @@
   // check if line is vertical
   if(j1==j2){
     for(i=minimum(i1,i2);i<=maximum(i1,i2);i++)
-     operator()(i,j1) = color ;
+     this->operator()(i,j1) = color ;
     return ;
   }
   mx = (double)(i1-i2)/(double)(j1-j2) ;
@@ -91,13 +91,13 @@
     if(i1>i2){
       for(i=i1;i>=i2;i--){
 	j = int(((double)i-b)/mx) ;
-	operator()(i,j) = color ;
+	this->operator()(i,j) = color ;
       }
     }
     else{
       for(i=i1;i<=i2;i++){
 	j = (int)((i-b)/mx) ;
-	operator()(i,j) = color ;
+	this->operator()(i,j) = color ;
       }
     }
   }
@@ -105,13 +105,13 @@
     if(j1>j2){
       for(j=j1;j>=j2;j--){
 	i = (int)(mx*j+b) ;
-	operator()(i,j) = color ;
+	this->operator()(i,j) = color ;
       }
     }
     else{
       for(j=j1;j<=j2;j++){
 	i = (int)(mx*j+b) ;
-	operator()(i,j) = color ;
+	this->operator()(i,j) = color ;
       }
     }
   }
@@ -136,9 +136,9 @@
 void MatrixImage<T>::drawPoint(int i, int j, double r , T color){
   for(int y=i-int(ceil(r)) ; y<i+int(ceil(r)) ; y++)
     for(int x = j-int(ceil(r)) ; x<j+int(ceil(r)) ; x++){
-      if(y>=0 && y<rows() && x>=0 && x<cols()){
+      if(y>=0 && y<this->rows() && x>=0 && x<this->cols()){
 	if( ((y-i)*(y-i)+(x-j)*(x-j))<= r*r)
-	  operator()(y,x) = color ;
+	  this->operator()(y,x) = color ;
       }
     }
 }
@@ -156,14 +156,14 @@
 */
 template <class T>
 void MatrixImage<T>::store(Matrix<T>& a){
-  if(a.rows() != rows() || a.cols() != cols()) {
-    a.resize(rows(),cols()) ;
+  if(a.rows() != this->rows() || a.cols() != this->cols()) {
+    a.resize(this->rows(),this->cols()) ;
   }
   T *aptr, *bptr ;
   int size,i ;
   aptr = &a(0,0)-1 ;
-  bptr = m-1 ;
-  size = cols()*rows() ;
+  bptr = this->m-1 ;
+  size = this->cols()*this->rows() ;
   for(i=0;i<size;i++)
     *(++aptr) = *(++bptr) ;  
 }
Index: matrix/cvector.h
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/matrix/cvector.h,v
retrieving revision 1.2
diff -u -r1.2 cvector.h
--- matrix/cvector.h	13 May 2002 21:07:45 -0000	1.2
+++ matrix/cvector.h	22 Jan 2005 09:18:25 -0000
@@ -54,10 +54,10 @@
     CVector(const BasicArray<T>& v) : Vector<T>(v), index(0)  {;}
     virtual ~CVector() {}
     
-    T& operator[](const int i) { return x[i%sze]; }
-    T  operator[](const int i) const   { return x[i%sze]; }
+    T& operator[](const int i) { return this->x[i%this->sze]; }
+    T  operator[](const int i) const   { return this->x[i%this->sze]; }
     
-    void put(T v) { x[index] = v ; index = (index+1)%sze; }
+    void put(T v) { this->x[index] = v ; index = (index+1)%this->sze; }
     
   protected:
     int index ;
Index: matrix/matrix.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/matrix/matrix.cpp,v
retrieving revision 1.2
diff -u -r1.2 matrix.cpp
--- matrix/matrix.cpp	13 May 2002 21:07:45 -0000	1.2
+++ matrix/matrix.cpp	22 Jan 2005 09:18:25 -0000
@@ -54,19 +54,19 @@
   if ( this == &a )
     return *this;
   
-  if ( a.rows() != rows() || a.cols() != cols() ){
+  if ( a.rows() != this->rows() || a.cols() != this->cols() ){
     resize(a.rows(),a.cols()) ;
   }
   
-  int sze = rows()*cols() ;
+  int sze = this->rows()*this->cols() ;
   T *ptr, *aptr ;
-  ptr = m-1 ;
+  ptr = this->m-1 ;
   aptr = a.m-1 ;
   
   for (i = sze; i > 0; --i)
     *(++ptr) = *(++aptr) ;
   
-  by_columns = a.by_columns;
+  this->by_columns = a.by_columns;
   
   return *this;
 }
@@ -100,10 +100,10 @@
 {
   int rwz,coz,i,j;
   
-  if ( rows() % a.rows() != 0 || cols() % a.cols() != 0 || rows() < a.rows() || cols() < a.cols() )
+  if ( this->rows() % a.rows() != 0 || this->cols() % a.cols() != 0 || this->rows() < a.rows() || this->cols() < a.cols() )
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize2D(rows(),cols(),a.rows(),a.cols()) ;
+      throw WrongSize2D(this->rows(),this->cols(),a.rows(),a.cols()) ;
 #else
       Error error("Matrix<T>::submatrix");
       error << "Matrix and submatrix incommensurate" ;
@@ -111,13 +111,13 @@
 #endif
     }
   
-  if ( sr >= rows()/a.rows() || sr < 0 || sc >= cols()/a.cols() || sc < 0 )
+  if ( sr >= this->rows()/a.rows() || sr < 0 || sc >= this->cols()/a.cols() || sc < 0 )
     {
 #ifdef USE_EXCEPTION
-      throw OutOfBound2D(sr,sc,0,rows()/a.rows()-1,0,cols()/a.cols()-1) ;
+      throw OutOfBound2D(sr,sc,0,this->rows()/a.rows()-1,0,this->cols()/a.cols()-1) ;
 #else
       Error error("Matrix<T>::submatrix");
-      error << "Submatrix location out of bounds.\nrowblock " << sr << ", " << rows()/a.rows() << " colblock " << sc << ", " << a.cols() << endl ;
+      error << "Submatrix location out of bounds.\nrowblock " << sr << ", " << this->rows()/a.rows() << " colblock " << sc << ", " << a.cols() << endl ;
       error.fatal() ;
 #endif
     }
@@ -127,13 +127,13 @@
 #ifdef COLUMN_ORDER
   for ( i = a.rows()-1; i >= 0; --i )
     for(j=a.cols()-1;j>=0;--j)
-      elem(i+rwz,j+coz) = a(i,j) ;
+      this->elem(i+rwz,j+coz) = a(i,j) ;
 #else
   T *ptr, *aptr ;
   aptr = a.m - 1;
   for ( i = a.rows()-1; i >= 0; --i )
     {
-      ptr = &m[(i+rwz)*cols()+coz]-1 ;
+      ptr = &this->m[(i+rwz)*this->cols()+coz]-1 ;
       for ( j = a.cols(); j > 0; --j)
 	*(++ptr) = *(++aptr) ;
     }  
@@ -159,7 +159,7 @@
   // Assign matrix a to this matrix at (i,j)
   int i, j;
   
-  if ( (rw + a.rows()) > rows() || ( cl + a.cols()) > cols()) {
+  if ( (rw + a.rows()) > this->rows() || ( cl + a.cols()) > this->cols()) {
 #ifdef USE_EXCEPTION
     throw MatrixErr();
 #else
@@ -172,12 +172,12 @@
 #ifdef COLUMN_ORDER
   for(i=0;i<a.rows();++i)
     for(j=0;j<a.cols();++j)
-      elem(i+rw,j+cl) = a(i,j) ;
+      this->elem(i+rw,j+cl) = a(i,j) ;
 #else
   T *pptr,*aptr ;
   aptr = a.m-1 ;
   for ( i = 0; i<a.rows(); ++i) {
-    pptr = &m[(i+rw)*cols()+cl]-1 ;
+    pptr = &this->m[(i+rw)*this->cols()+cl]-1 ;
     for ( j = 0; j < a.cols(); ++j)
       *(++pptr) = *(++aptr);
   }
@@ -208,7 +208,7 @@
 Matrix<T> Matrix<T>::get(int rw, int cl, int nr, int nc) const
 {
   Matrix<T> getmat(nr,nc) ;
-  if ( (rw+nr) > rows() || (cl+nc) > cols()) {
+  if ( (rw+nr) > this->rows() || (cl+nc) > this->cols()) {
 #ifdef USE_EXCEPTION
     throw MatrixErr();
 #else
@@ -223,12 +223,12 @@
 #ifdef COLUMN_ORDER
   for(i=0;i<nr;++i)
     for(j=0;j<nc;++j)
-      getmat(i,j) = elem(i+rw,j+cl) ;
+      getmat(i,j) = this->elem(i+rw,j+cl) ;
 #else
   T *pptr,*aptr ;
   aptr = getmat.m-1;
   for (i = 0; i < nr; ++i) {
-    pptr = &m[(i+rw)*cols()+cl]-1 ;
+    pptr = &this->m[(i+rw)*this->cols()+cl]-1 ;
     for ( j = 0; j < nc; ++j)
       *(++aptr) = *(++pptr) ;
   }
@@ -252,11 +252,11 @@
   double sum, maxsum;
   int init=0 ;
   T *pptr ;
-  pptr = m-1 ;
+  pptr = this->m-1 ;
   maxsum = 0 ; // Silence the warning message
-  for(i=0;i<rows();++i){
+  for(i=0;i<this->rows();++i){
     sum = 0 ;
-    for ( j = 0; j < cols(); ++j) 
+    for ( j = 0; j < this->cols(); ++j) 
       sum += *(++pptr) ;
     if(init)
       maxsum = (maxsum>sum) ? maxsum : sum;
@@ -285,12 +285,12 @@
 {
   int i, iend;
   
-  iend = rows();
-  if ( iend > cols() )
-    iend = cols();
+  iend = this->rows();
+  if ( iend > this->cols() )
+    iend = this->cols();
   
   for (i = iend-1; i >=0; --i)
-    elem(i,i) = a;
+    this->elem(i,i) = a;
 
 }
 
@@ -308,10 +308,10 @@
 template <class T>
 Vector<T> Matrix<T>::getDiag(){
   int i, iend;
-  Vector<T> vec(minimum(rows(),cols())) ;
-  iend = minimum(rows(),cols());
+  Vector<T> vec(minimum(this->rows(),this->cols())) ;
+  iend = minimum(this->rows(),this->cols());
   for (i = iend-1; i >=0; --i)
-      vec[i] = elem(i,i);
+      vec[i] = this->elem(i,i);
   return vec ;
 }
 
@@ -328,8 +328,8 @@
 Matrix<T>& Matrix<T>::operator+=(double a)
 {
   T *p1 ;
-  p1 = m-1 ;
-  const int size = rows()*cols() ;
+  p1 = this->m-1 ;
+  const int size = this->rows()*this->cols() ;
   for(int i=size; i>0; --i)
     *(++p1) += a ;  
   return *this ;
@@ -348,8 +348,8 @@
 Matrix<T>& Matrix<T>::operator-=(double a)
 {
   T *p1 ;
-  p1 = m-1 ;
-  const int size = rows()*cols() ;
+  p1 = this->m-1 ;
+  const int size = this->rows()*this->cols() ;
   for(int i=size; i>0; --i)
     *(++p1) -= a ;  
   return *this ;
@@ -368,8 +368,8 @@
 Matrix<T>& Matrix<T>::operator*=(double a)
 {
   T *p1 ;
-  p1 = m-1 ;
-  const int size = rows()*cols() ;
+  p1 = this->m-1 ;
+  const int size = this->rows()*this->cols() ;
   for(int i=size; i>0; --i)
     *(++p1) *= a ;  
   return *this ;
@@ -388,8 +388,8 @@
 Matrix<T>& Matrix<T>::operator/=(double a)
 {
   T *p1 ;
-  p1 = m-1 ;
-  const int size = rows()*cols() ;
+  p1 = this->m-1 ;
+  const int size = this->rows()*this->cols() ;
   for(int i=size; i>0; --i)
     *(++p1) /= a ;  
   return *this ;
@@ -408,15 +408,15 @@
 template <class T> 
 Matrix<T>& Matrix<T>::operator+=(const Matrix<T> &a)
 {
-  if ( a.rows() != rows() || a.cols() != cols() )
+  if ( a.rows() != this->rows() || a.cols() != this->cols() )
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize2D(rows(),cols(),a.rows(),a.cols());
+      throw WrongSize2D(this->rows(),this->cols(),a.rows(),a.cols());
 #else
       Error error("Matrix<T>::operator+=") ;
-      if ( rows() != a.rows() )
+      if ( this->rows() != a.rows() )
 	error << "Matrices are of diferent size, a.rows() = " << rows() << " and b.rows() = " << a.rows() << endl ;
-      if ( cols() != a.cols())
+      if ( this->cols() != a.cols())
 	error << "Matrices are of diferent size, a.cols() = " << cols() << " and b.cols() = " << a.cols() << endl ;
       error.fatal() ;
 #endif
@@ -425,8 +425,8 @@
   int i, sze ;
   T *aptr,*sptr ;
   aptr = a.m - 1 ;
-  sptr = m - 1 ;
-  sze = rows()*cols() ;
+  sptr = this->m - 1 ;
+  sze = this->rows()*this->cols() ;
   for (i = sze; i > 0; --i){
       *(++sptr) += *(++aptr) ;
   }
@@ -468,15 +468,15 @@
 template <class T> 
 Matrix<T>& Matrix<T>::operator-=(const Matrix<T> &a)
 {
-  if ( a.rows() != rows() || a.cols() != cols() )
+  if ( a.rows() != this->rows() || a.cols() != this->cols() )
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize2D(rows(),cols(),a.rows(),a.cols());
+      throw WrongSize2D(this->rows(),this->cols(),a.rows(),a.cols());
 #else
       Error error("Matrix<T>::operator-=") ;
-      if ( rows() != a.rows() )
+      if ( this->rows() != a.rows() )
 	error << "Matrices are of diferent size, a.rows() = " << rows() << " and b.rows() = " << a.rows() << endl ;
-      if ( cols() != a.cols())
+      if ( this->cols() != a.cols())
 	error << "Matrices are of diferent size, a.cols() = " << cols() << " and b.cols() = " << a.cols() << endl ;
       error.fatal() ;
 #endif
@@ -485,8 +485,8 @@
   int i, size;
   T *aptr,*sptr ;
   aptr = a.m - 1 ;
-  sptr = m - 1 ;
-  size = rows()*cols() ;
+  sptr = this->m - 1 ;
+  size = this->rows()*this->cols() ;
   for (i = size; i > 0; --i){
       *(++sptr) -= *(++aptr) ;
   }
@@ -742,14 +742,14 @@
 template <class T>
 T Matrix<T>::trace() const
 {
-  int size = rows();
+  int size = this->rows();
   T sum = (T)0;
   
-  if ( size > cols() )
-    size = cols();
+  if ( size > this->cols() )
+    size = this->cols();
   
   for (int d = 0; d < size; ++d)
-    sum += elem(d,d) ;
+    sum += this->elem(d,d) ;
   
   return sum;
 }
@@ -770,12 +770,12 @@
 template <class T>
 Matrix<T> Matrix<T>::herm() const
 {
-  int i, j, r = cols(), c = rows();
+  int i, j, r = this->cols(), c = this->rows();
   Matrix<T> adj(r,c);
   
   for (i = 0; i < r; ++i)
     for (j = 0; j < c; ++j)
-      adj.elem(i,j) = elem(j,i) ;
+      adj.elem(i,j) = this->elem(j,i) ;
 
   return adj;
 
@@ -794,11 +794,11 @@
 template <class T>
 Matrix<T> Matrix<T>::flop() const
 {					
-  Matrix<T> f(rows(),cols()) ;
-  for(int i=rows()-1;i>=0;--i)
-    for(int j=cols()-1;j>=0;--j)
+  Matrix<T> f(this->rows(),this->cols()) ;
+  for(int i=this->rows()-1;i>=0;--i)
+    for(int j=this->cols()-1;j>=0;--j)
       {
-	f(i,j) = elem(i,cols()-j-1);
+	f(i,j) = this->elem(i,this->cols()-j-1);
       }
   return f; 
 }
@@ -817,13 +817,13 @@
 {					
   // same as hermitian for real Matrix<T>
   int i, j;
-  const int& r = cols();
-  const int& c = rows();
+  const int& r = this->cols();
+  const int& c = this->rows();
   Matrix<T> adj(r,c);
   
   for (i = r-1; i >=0; --i)
     for (j = c-1; j >=0; --j)
-      adj.elem(i,j) = elem(j,i) ;
+      adj.elem(i,j) = this->elem(j,i) ;
   
   
   return adj; 
@@ -844,7 +844,7 @@
 int Matrix<T>::read(char* filename) {
   ifstream fin(filename) ;
   if(!fin) {
-    resize(1,1) ;
+    this->resize(1,1) ;
     return 0 ;
   }
   int r,c ;
@@ -855,8 +855,8 @@
   if(r) return 0 ;
   if(!fin.read((char*)&r,sizeof(int))) return 0 ;
   if(!fin.read((char*)&c,sizeof(int))) return 0 ;
-  resize(r,c) ;
-  if(!fin.read((char*)m,sizeof(T)*r*c)) return 0 ;
+  this->resize(r,c) ;
+  if(!fin.read((char*)this->m,sizeof(T)*r*c)) return 0 ;
 
   delete []type ;
   return 1 ;
@@ -877,11 +877,11 @@
 int Matrix<T>::read(char* filename,int r, int c) {
   ifstream fin(filename) ;
   if(!fin) {
-    resize(1,1) ;
+    this->resize(1,1) ;
     return 0 ;
   }
-  resize(r,c) ;
-  if(!fin.read((char*)m,sizeof(T)*r*c)) return 0 ;
+  this->resize(r,c) ;
+  if(!fin.read((char*)this->m,sizeof(T)*r*c)) return 0 ;
 
   return 1 ;
 }
@@ -904,11 +904,11 @@
   if(!fout)
     return 0 ;
   int r,c ;
-  r = rows() ; c = cols() ;
+  r = this->rows() ; c = this->cols() ;
   if(!fout.write((char*)&"matrix",sizeof(char)*6)) return 0 ;
   if(!fout.write((char*)&r,sizeof(int))) return 0 ;
   if(!fout.write((char*)&c,sizeof(int))) return 0 ;
-  if(!fout.write((char*)m,sizeof(T)*r*c)) return 0 ;
+  if(!fout.write((char*)this->m,sizeof(T)*r*c)) return 0 ;
   return 1;
 }
 
@@ -927,7 +927,7 @@
   ofstream fout(filename) ;
   if(!fout)
     return 0 ;
-  if(!fout.write((char*)m,sizeof(T)*rows()*cols())) return 0 ;
+  if(!fout.write((char*)this->m,sizeof(T)*this->rows()*this->cols())) return 0 ;
   return 1;
 }
 
Index: matrix/vector.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/matrix/vector.cpp,v
retrieving revision 1.2
diff -u -r1.2 vector.cpp
--- matrix/vector.cpp	13 May 2002 21:07:45 -0000	1.2
+++ matrix/vector.cpp	22 Jan 2005 09:18:25 -0000
@@ -51,16 +51,16 @@
   if(this==&b)
     return *this ;
 
-  if ( n() != b.n())
+  if ( this->n() != b.n())
     {
       resize(b.n()) ;
     }
 
-  sze = b.n() ;
+  this->sze = b.n() ;
   T *pa, *pb ;
-  pa = x-1 ;
+  pa = this->x-1 ;
   pb = b.x-1 ;
-  for(int i=n();i>0;--i){
+  for(int i=this->n();i>0;--i){
     *(++pa) = *(++pb) ;
   }
   return *this;
@@ -79,13 +79,13 @@
 template <class T>
 Vector<T>& Vector<T>::operator=(const BasicArray<T> &b)
 {
-  if ( size() != b.size())
+  if ( this->size() != b.size())
     {
       resize(b.size()) ;
     }
   T *ptr ;
-  ptr = x - 1 ;
-  for(int i=size()-1;i>=0;--i)
+  ptr = this->x - 1 ;
+  for(int i=this->size()-1;i>=0;--i)
      *(++ptr) = b[i] ;
 
   return *this;
@@ -105,9 +105,9 @@
 template <class T>
 T Vector<T>::operator=(const T d)
 {
-  const int sz = size(); 
+  const int sz = this->size(); 
   T *ptr ;
-  ptr = x-1 ;
+  ptr = this->x-1 ;
   for (int i = sz; i > 0; --i)
     *(++ptr) = d ;
 
@@ -130,19 +130,19 @@
 template <class T>
 Vector<T>& Vector<T>::operator+=(const Vector<T> &a)
 {
-  if ( a.size() != size())
+  if ( a.size() != this->size())
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize(size(),a.size()) ;
+      throw WrongSize(this->size(),a.size()) ;
 #else
       Error error("Vector<T>::operator+=(Vector<T>&)");
-      error << "Vector<T> a += Vector<T> b different sizes, a = " << size() << ", b = " << a.size() ;
+      error << "Vector<T> a += Vector<T> b different sizes, a = " << this->size() << ", b = " << a.size() ;
       error.fatal() ;
 #endif
     }
-  const int sz = size();
+  const int sz = this->size();
   T *ptr,*aptr ;
-  ptr = x-1 ;
+  ptr = this->x-1 ;
   aptr = a.x-1 ;
   for (int i = sz; i >0; --i)
     *(++ptr) += *(++aptr) ;
@@ -165,10 +165,10 @@
 template <class T>
 Vector<T>& Vector<T>::operator-=(const Vector<T> &a)
 {
-  if ( a.size() != size())
+  if ( a.size() != this->size())
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize(size(),a.size()) ;
+      throw WrongSize(this->size(),a.size()) ;
 #else
       Error error("Vector<T>::operator-=(Vector<T>&)");
       error << "Vector<T> a -= Vector<T> b different sizes, a = " << size() << ", b = " << a.size() ;
@@ -176,9 +176,9 @@
 #endif
     }
   
-  const int sz = size(); 
+  const int sz = this->size(); 
   T *ptr,*aptr ;
-  ptr = x-1 ;
+  ptr = this->x-1 ;
   aptr = a.x-1 ;
   for (int i = sz; i > 0; --i)
     *(++ptr) -= *(++aptr) ;
@@ -391,7 +391,7 @@
     }
 
   T *aptr,*bptr ;
-  aptr = &x[i]-1 ;
+  aptr = &this->x[i]-1 ;
   bptr = b.x-1 ;
   for ( int j = b.rows(); j > 0; --j)
       *(++aptr) = *(++bptr) ;
@@ -429,7 +429,7 @@
 
   Vector<T> subvec(l) ;
   T *aptr, *bptr ;
-  aptr = &x[i] - 1 ;
+  aptr = &this->x[i] - 1 ;
   bptr = subvec.x -1 ;
   for ( int j = l; j > 0; --j)
     *(++bptr) = *(++aptr) ;
@@ -449,12 +449,12 @@
 */
 template <class T>
 int Vector<T>::minIndex() const {
-  T min = x[0] ;
+  T min = this->x[0] ;
   int index = 0 ;
 
-  for(int i=1;i<n();i++){
-    if(x[i]<=min){
-      min = x[i] ;
+  for(int i=1;i<this->n();i++){
+    if(this->x[i]<=min){
+      min = this->x[i] ;
       index = i ;
     }
   }
@@ -523,12 +523,12 @@
   T a ;
   T *v1,*v2  ;
 
-  ir = sze-1 ;
+  ir = this->sze-1 ;
   l = 0 ;
   
   while(1){
     if(ir-l<M){ // perform an insertion sort when the array is small enough
-      v1 = &x[l] ;
+      v1 = &this->x[l] ;
       for(j=l+1;j<=ir;++j){
 	a = *(++v1) ;
 	v2 = v1 ;
@@ -547,31 +547,31 @@
     }
     else{
       k=(l+ir) >> 1 ;
-      swap(x[k],x[l+1]) ;
-      if(x[l+1] > x[ir]){
-	swap(x[l+1],x[ir]) ;
+      swap(this->x[k],this->x[l+1]) ;
+      if(this->x[l+1] > this->x[ir]){
+	swap(this->x[l+1],this->x[ir]) ;
       }
-      if(x[l]> x[ir]){
-	swap(x[l],x[ir]) ;
+      if(this->x[l]> this->x[ir]){
+	swap(this->x[l],this->x[ir]) ;
       }
-      if(x[l+1] > x[l]){
-	swap(x[l+1],x[l]) ;
+      if(this->x[l+1] > this->x[l]){
+	swap(this->x[l+1],this->x[l]) ;
       }
       i=l+1 ;
       j=ir ;
-      a=x[l] ;
-      v1 = &x[i] ;
-      v2 = &x[j] ;
+      a=this->x[l] ;
+      v1 = &this->x[i] ;
+      v2 = &this->x[j] ;
       while(1){
 	while(*v1 < a) { ++i ; ++v1 ; }
 	while(*v2 > a) { --j ; --v2 ; }
 	if(j<i) break ;
 	if(*v1 == *v2)  // both are equal to a...
 	  break ;
-	swap(x[i],x[j]) ;
+	swap(this->x[i],this->x[j]) ;
       }
-      x[l] = x[j] ;
-      x[j] = a ;
+      this->x[l] = this->x[j] ;
+      this->x[j] = a ;
       jstack += 2 ;
       if(jstack>=Nstack){
 	istack.resize(istack.n()+Nstack) ; // increase the vector size
@@ -618,10 +618,10 @@
   int jstack=0;
   T a ;
 
-  ir = sze-1 ;
+  ir = this->sze-1 ;
   l = 0 ;
   
-  index.resize(sze) ;
+  index.resize(this->sze) ;
   for(i=0;i<index.n();++i)
     index[i] = i ;
 
@@ -629,9 +629,9 @@
     if(ir-l<M){ // perform an insertion sort when the array is small enough
       for(j=l+1;j<=ir;++j){
 	indext = index[j] ;
-	a = x[indext] ;
+	a = this->x[indext] ;
 	for(i=j-1;i>=0;--i){
-	  if(x[index[i]] <= a) break ;
+	  if(this->x[index[i]] <= a) break ;
 	  index[i+1] = index[i] ;
 	}
 	index[i+1] = indext ;
@@ -643,24 +643,24 @@
     else{
       k=(l+ir) >> 1 ;
       swap(index[k],index[l+1]) ;
-      if(x[index[l+1]] > x[index[ir]]){
+      if(this->x[index[l+1]] > this->x[index[ir]]){
 	swap(index[l+1],index[ir]) ;
       }
-      if(x[index[l]]> x[index[ir]]){
+      if(this->x[index[l]]> this->x[index[ir]]){
 	swap(index[l],index[ir]) ;
       }
-      if(x[index[l+1]] > x[index[l]]){
+      if(this->x[index[l+1]] > this->x[index[l]]){
 	swap(index[l+1],index[l]) ;
       }
       i=l+1 ;
       j=ir ;
       indext = index[l] ;
-      a=x[indext] ;
+      a=this->x[indext] ;
       while(1){
-	while(x[index[i]] < a) { ++i ; }
-	while(x[index[j]] > a) { --j ; }
+	while(this->x[index[i]] < a) { ++i ; }
+	while(this->x[index[j]] > a) { --j ; }
 	if(j<i) break ;
-	if(x[index[i]] == x[index[j]])
+	if(this->x[index[i]] == this->x[index[j]])
 	  break ;
 	swap(index[i],index[j]) ;
       }
Index: matrix/vector.h
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/matrix/vector.h,v
retrieving revision 1.2
diff -u -r1.2 vector.h
--- matrix/vector.h	13 May 2002 21:07:45 -0000	1.2
+++ matrix/vector.h	22 Jan 2005 09:18:25 -0000
@@ -69,7 +69,7 @@
   {
   public:
     int rows() const //!< a reference to the size of the vector
-      { return sze ;}
+      { return this->sze ;}
     Vector() : BasicArray<T>(1) {} //!< Basic constructor
     Vector(const int r) : BasicArray<T>(r) {}
     Vector(const Vector<T>& v) : BasicArray<T>(v) {}
Index: morph/kernel.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/morph/kernel.cpp,v
retrieving revision 1.1
diff -u -r1.1 kernel.cpp
--- morph/kernel.cpp	27 Jan 2003 11:37:36 -0000	1.1
+++ morph/kernel.cpp	22 Jan 2005 09:18:25 -0000
@@ -56,10 +56,10 @@
     error.fatal();
 #endif
   }
-  resize(r,c);
-  m_center_row = center_r;
-  m_center_column = center_c;
-  m_mask.resize(rows(),cols());
+  this->resize(r,c);
+  this->m_center_row = center_r;
+  this->m_center_column = center_c;
+  m_mask.resize(this->rows(),this->cols());
   m_mask.reset(255);
 }
 
@@ -86,20 +86,20 @@
 PLib::Morph::Kernel<T>::Kernel(const T k00, const T k01, const T k02, 
 			       const T k10, const T k11, const T k12, 
 			       const T k20, const T k21, const T k22){
-  resize(3,3);
-  m_mask.resize(rows(),cols());
+  this->resize(3,3);
+  m_mask.resize(this->rows(),this->cols());
   m_mask.reset(255);
   m_center_row = 1;
   m_center_column = 1;
-  elem(0,0) = k00;
-  elem(0,1) = k01;
-  elem(0,2) = k02;
-  elem(1,0) = k10;
-  elem(1,1) = k11;
-  elem(1,2) = k12;
-  elem(2,0) = k20;
-  elem(2,1) = k21;
-  elem(2,2) = k22;
+  this->elem(0,0) = k00;
+  this->elem(0,1) = k01;
+  this->elem(0,2) = k02;
+  this->elem(1,0) = k10;
+  this->elem(1,1) = k11;
+  this->elem(1,2) = k12;
+  this->elem(2,0) = k20;
+  this->elem(2,1) = k21;
+  this->elem(2,2) = k22;
 }
 
 
@@ -117,12 +117,12 @@
 */
 template <class T>
 void PLib::Morph::DiskKernel<T>::init(int r){
-  resize(2*r+1,2*r+1);
-  m_center_row = r;
-  m_center_column = r;
+  this->resize(2*r+1,2*r+1);
+  this->m_center_row = r;
+  this->m_center_column = r;
 
-  m_mask.resize(rows(),cols());
-  m_mask.reset(0); 
+  this->m_mask.resize(this->rows(),this->cols());
+  this->m_mask.reset(0); 
 
   for(int i=0;i<=r;++i){
     circleMidpoint(i,setValueAtRadius(i));
@@ -142,10 +142,10 @@
 */
 template <class T>
 void PLib::Morph::DiskKernel<T>::setElement(int r, int c, T value){
-  elem(m_center_row + r, m_center_column + c) = value;
-  m_mask(m_center_row + r, m_center_column + c) = 255;
+  this->elem(this->m_center_row + r, this->m_center_column + c) = value;
+  this->m_mask(this->m_center_row + r, this->m_center_column + c) = 255;
 
-  if(r>1 && elem(m_center_row+r-1,m_center_column+c)==0){
+  if(r>1 && this->elem(this->m_center_row+r-1,this->m_center_column+c)==0){
     double radius = sqrt( (double)(r*r) + (double)(c*c) );
     setCirclePoints(r-1,c,setValueAtRadius(radius));
   }
Index: morph/kernel.h
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/morph/kernel.h,v
retrieving revision 1.1
diff -u -r1.1 kernel.h
--- morph/kernel.h	27 Jan 2003 11:37:36 -0000	1.1
+++ morph/kernel.h	22 Jan 2005 09:18:25 -0000
@@ -107,36 +107,36 @@
     protected:
       DiskKernel(){;}
 
-      void init(int radius);
-      void setCirclePoints(int x, int y, T value);
-      void circleMidpoint(int radius, T value);
-      void setElement(int r, int c, T value);
-      virtual T setValueAtRadius(double radius);
-
-    private:
-      T m_value;
-    };
-
-  /*! 
-    \brief a disk kernel with values as a gaussian
-
-    This kernel creates a disk. It sets the mask appropriately for
-    gray level operations. The values inside the disk are 
-    gaussian distributed from the center (max value) to 
-    the radius (min value).
-
-    \author Philippe Lavoie
-    \date 23 Jan 2003
-  */
-  template <class T>
-    class GaussianDiskKernel: public DiskKernel<T>
-    {
-    public:
-      GaussianDiskKernel(int radius,T min, T max){
-	m_min = min;
-	m_max = max;
-	m_radius = radius;
-	init(radius);
+	      void init(int radius);
+	      void setCirclePoints(int x, int y, T value);
+	      void circleMidpoint(int radius, T value);
+	      void setElement(int r, int c, T value);
+	      virtual T setValueAtRadius(double radius);
+
+	    private:
+	      T m_value;
+	    };
+
+	  /*! 
+	    \brief a disk kernel with values as a gaussian
+
+	    This kernel creates a disk. It sets the mask appropriately for
+	    gray level operations. The values inside the disk are 
+	    gaussian distributed from the center (max value) to 
+	    the radius (min value).
+
+	    \author Philippe Lavoie
+	    \date 23 Jan 2003
+	  */
+	  template <class T>
+	    class GaussianDiskKernel: public DiskKernel<T>
+	    {
+	    public:
+	      GaussianDiskKernel(int radius,T min, T max){
+		m_min = min;
+		m_max = max;
+		m_radius = radius;
+		this->init(radius);
       }
 
     protected:
@@ -165,7 +165,7 @@
 	m_min = min;
 	m_max = max;
 	m_radius = radius;
-	init(radius);
+	this->init(radius);
       }
 
     protected:
Index: morph/watershed.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/morph/watershed.cpp,v
retrieving revision 1.3
diff -u -r1.3 watershed.cpp
--- morph/watershed.cpp	15 May 2003 02:34:55 -0000	1.3
+++ morph/watershed.cpp	22 Jan 2005 09:18:25 -0000
@@ -29,10 +29,11 @@
 #include "watershed.h"
 #include "../matrix/barray.h"
 #include "../matrix/barray2d.h"
-#include <hash_map>
+#include <ext/hash_map>
 #include <alloc.h>
+#include <algorithm>
 
-typedef std::pair <int, int> HashPair;
+typedef __gnu_cxx::pair <int, int> HashPair;
 
 
 /*!
@@ -67,11 +68,11 @@
   }
   return min_value;
 }
-void setToEquivalentLabel(int i, std::hash_map<int,int>& eq_table, PLib::Basic2DArray<int>& labels){
-  std::hash_map<int,int>::iterator eq_table_iterator;
-  std::hash_map<int,int>::const_iterator value;
-  std::hash_map<int,int>::const_iterator last_value;
-  std::hash_map<int,int> eq_result;
+void setToEquivalentLabel(int i, __gnu_cxx::hash_map<int,int>& eq_table, PLib::Basic2DArray<int>& labels){
+  __gnu_cxx::hash_map<int,int>::iterator eq_table_iterator;
+  __gnu_cxx::hash_map<int,int>::const_iterator value;
+  __gnu_cxx::hash_map<int,int>::const_iterator last_value;
+  __gnu_cxx::hash_map<int,int> eq_result;
 
   PLib::BasicArray<int> equivalence_stack(labels.cols());
   equivalence_stack.resize(0);
@@ -139,7 +140,7 @@
   PLib::BasicArray<int> local_labels(9);
   labels.resize(image.rows(),image.cols());
 
-  std::hash_map<int,int> eq_table;
+  __gnu_cxx::hash_map<int,int> eq_table;
 
   // Top down pass
   for(int i=0;i<image.rows();++i){
@@ -219,8 +220,8 @@
   \author Philippe Lavoie
   \date 27 Jan 2003
 */
-template <>
-void PLib::Morph::watershed<unsigned char> (const PLib::Basic2DArray<unsigned char>& image, PLib::Basic2DArray<unsigned char>& result){
+template <class T>
+void PLib::Morph::watershed(const PLib::Basic2DArray<T>& image, PLib::Basic2DArray<T>& result){
   PLib::Basic2DArray<unsigned char> resolved_mask(image.rows(),image.cols());
   PLib::Basic2DArray<int> labels(image.rows(),image.cols());
   result.resize(image.rows(), image.cols());
@@ -244,19 +245,20 @@
 
 #ifdef NO_IMPLICIT_TEMPLATES
 
-namespace std {
+namespace __gnu_cxx {
   template class hashtable<pair<int const, int>, int, hash<int>, _Select1st<pair<int const, int> >, equal_to<int>, allocator<int> >;
-  template unsigned long const * __lower_bound<unsigned long const *, unsigned long, int>(unsigned long const *, unsigned long const *, unsigned long const &, int *);
-  template _Hashtable_node<pair<int const, int> > ** fill_n<_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > *>(_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > * const &);
+//  template unsigned long const * __lower_bound<unsigned long const *, unsigned long, int>(unsigned long const *, unsigned long const *, unsigned long const &, int *);
+//  template _Hashtable_node<pair<int const, int> > ** fill_n<_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > *>(_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > * const &);
   template class allocator<int>;
-  template class vector<_Hashtable_node<pair<int const, int> > *, allocator<int> >;
-  template void fill<_Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > *>(_Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > * const &);
-  template class _Hashtable_iterator<pair<int const, int>, int, hash<int>, _Select1st<pair<int const, int> >, equal_to<int>, allocator<int> >;
-  //  template class vector<_Hashtable_node<pair<int const, int> > *, allocator<int> >::insert(_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > *const &);
+//  template class vector<_Hashtable_node<pair<int const, int> > *, allocator<int> >;
+//  template void fill<_Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > *>(_Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > **, _Hashtable_node<pair<int const, int> > * const &);
+//  template class _Hashtable_iterator<pair<int const, int>, int, hash<int>, _Select1st<pair<int const, int> >, equal_to<int>, allocator<int> >;
+//  template class vector<_Hashtable_node<pair<int const, int> > *, allocator<int> >::insert(_Hashtable_node<pair<int const, int> > **, unsigned int, _Hashtable_node<pair<int const, int> > *const &);
 }
 
 namespace PLib{
   namespace Morph{
+template void watershed<unsigned char> (const PLib::Basic2DArray<unsigned char>& image, PLib::Basic2DArray<unsigned char>& result);
   }
 }
 #endif
Index: numerical/matrixMat.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/numerical/matrixMat.cpp,v
retrieving revision 1.2
diff -u -r1.2 matrixMat.cpp
--- numerical/matrixMat.cpp	13 May 2002 21:07:45 -0000	1.2
+++ numerical/matrixMat.cpp	22 Jan 2005 09:18:25 -0000
@@ -48,9 +48,9 @@
 template <class T>
 LUMatrix<T>& LUMatrix<T>::operator=(const LUMatrix<T>& a){
   resize(a.rows(),a.cols()) ;
-  for(int i=0;i<rows();++i)
-    for(int j=0;j<cols();++j)
-      elem(i,j) = a(i,j) ;
+  for(int i=0;i<this->rows();++i)
+    for(int j=0;j<this->cols();++j)
+      this->elem(i,j) = a(i,j) ;
   pivot_ = a.pivot_ ;
   return *this ;
 }
@@ -90,7 +90,7 @@
   //	lu = a;	 must do it by copying or LUFACT will be recursively called !
   for(i=0;i<n;++i)
     for(j=0;j<n;++j)
-      elem(i,j) = a(i,j) ;
+      this->elem(i,j) = a(i,j) ;
 
   errval = 0;
   nm1 = n - 1;
@@ -129,24 +129,24 @@
 	    }
 	  pivot_[k] = l;
 
-	  if ( elem(l,k) != 0.0 )
+	  if ( this->elem(l,k) != 0.0 )
 	    {			// nonsingular pivot found 
 	      if (l != k ){	// interchange needed 
 		for (i = k; i < n; i++)
 		  {
-		    t = elem(l,i) ;
-		    elem(l,i) = elem(k,i) ;
-		    elem(k,i) = t ; 
+		    t = this->elem(l,i) ;
+		    this->elem(l,i) = this->elem(k,i) ;
+		    this->elem(k,i) = t ; 
 		  }
 		sign = -sign ;
 	      }
-	      q =  elem(k,k);	/* scale row */
+	      q =  this->elem(k,k);	/* scale row */
 	      for (i = kp1; i < n; i++)
 		{
-		  t = - elem(i,k)/q;
-		  elem(i,k) = t;
+		  t = - this->elem(i,k)/q;
+		  this->elem(i,k) = t;
 		  for (j = kp1; j < n; j++)
-		    elem(i,j) += t * elem(k,j);
+		    this->elem(i,j) += t * this->elem(k,j);
 		}
 	    }
 	  else		/* pivot singular */
@@ -156,7 +156,7 @@
     }
   
   pivot_[nm1] = nm1;
-  if (elem(nm1,nm1) == 0.0)
+  if (this->elem(nm1,nm1) == 0.0)
     errval = nm1;  
   return *this;
 }
@@ -166,7 +166,7 @@
 template <class T>
 Matrix<T> LUMatrix::operator Matrix()	
 {
-  int i, j, r = rows(), c = cols();
+  int i, j, r = this->rows(), c = this->cols();
 
   Matrix mat( r, c );
 	
@@ -196,9 +196,9 @@
 */
 template <class T>
 T LUMatrix<T>::determinant(){
-  T det = elem(0,0) ;
-  for(int i=1;i<rows();++i)
-    det *= elem(i,i) ;
+  T det = this->elem(0,0) ;
+  for(int i=1;i<this->rows();++i)
+    det *= this->elem(i,i) ;
   return det * (T)sign ;
 }
 
@@ -209,7 +209,7 @@
 void LUMatrix<T>::backSub(const Matrix<T>& B, Matrix<T>& X){
   int i,ii,ip,j,k ;
   T sum ;
-  int n = rows() ;
+  int n = this->rows() ;
   // one column at a time
   //X.resize(n,B.cols()) ;
   X = B ;
@@ -221,7 +221,7 @@
       X(ip,j) = X(i,j) ;
       if(ii)
 	for(k=ii;k<i;++k)
-	  sum -= elem(i,k)*X(k,j);
+	  sum -= this->elem(i,k)*X(k,j);
       else
 	if(sum)
 	  ii=i ;
@@ -230,8 +230,8 @@
     for(i=n-1;i>0;--i){ // doing forward substitution
       sum=X(i,j) ;
       for(k=i+1;k<n;++k)
-	sum -= elem(i,k)*X(k,j) ;
-      X(i,j) = sum/elem(i,i) ;
+	sum -= this->elem(i,k)*X(k,j) ;
+      X(i,j) = sum/this->elem(i,i) ;
     }
   }
 }
@@ -253,18 +253,18 @@
   T ten;
   int i, j, k, l, kb, kp1, nm1, n, coln;
 
-  if ( rows() != cols() )
+  if ( this->rows() != this->cols() )
     {
 #ifdef USE_EXCEPTION
-    throw WrongSize2D(rows(),cols(),0,0) ;
+    throw WrongSize2D(this->rows(),this->cols(),0,0) ;
 #else
       Error error("invm");
-      error << "matrix inverse, not square: " << rows() << " by " << cols() << endl;
+      error << "matrix inverse, not square: " << this->rows() << " by " << this->cols() << endl;
       error.fatal();
 #endif
     }
 
-  n = coln = rows();
+  n = coln = this->rows();
 
 
   inv = *this ;
@@ -338,13 +338,13 @@
 template <class T>
 Matrix<T> LUMatrix<T>::inverse() 
 {
-  if ( rows() != cols() )
+  if ( this->rows() != this->cols() )
     {
 #ifdef USE_EXCEPTION
-      throw WrongSize2D(rows(),cols(),0,0) ;
+      throw WrongSize2D(this->rows(),this->cols(),0,0) ;
 #else
       Error error("invm");
-      error << "matrix inverse, not square: " << rows() << " by " << cols() << endl;
+      error << "matrix inverse, not square: " << this->rows() << " by " << this->cols() << endl;
       error.fatal();
 #endif
     }
Index: nurbs/d_nurbsSub.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/d_nurbsSub.cpp,v
retrieving revision 1.2
diff -u -r1.2 d_nurbsSub.cpp
--- nurbs/d_nurbsSub.cpp	13 May 2002 21:07:45 -0000	1.2
+++ nurbs/d_nurbsSub.cpp	22 Jan 2005 09:18:25 -0000
@@ -14,8 +14,8 @@
   template class RenderMeshPoints<double> ;
 
   
-  double NurbSurface<double>::epsilon = 1e-6 ;
-  double SurfSample<double>::epsilon = 1e-6 ;
+  template <> double NurbSurface<double>::epsilon = 1e-6 ;
+  template <> double SurfSample<double>::epsilon = 1e-6 ;
 
   template void DrawSubdivision( NurbSurface<double> *, double tolerance );
   template void DrawEvaluation( NurbSurface<double> * );
Index: nurbs/f_nurbsSub.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/f_nurbsSub.cpp,v
retrieving revision 1.2
diff -u -r1.2 f_nurbsSub.cpp
--- nurbs/f_nurbsSub.cpp	13 May 2002 21:07:45 -0000	1.2
+++ nurbs/f_nurbsSub.cpp	22 Jan 2005 09:18:25 -0000
@@ -14,8 +14,8 @@
   template class RenderMeshPoints<float> ;
 
   
-  float NurbSurface<float>::epsilon = 1e-6 ;
-  float SurfSample<float>::epsilon = 1e-6 ;
+  template <> float NurbSurface<float>::epsilon = 1e-6 ;
+  template <> float SurfSample<float>::epsilon = 1e-6 ;
 
   template void DrawSubdivision( NurbSurface<float> *, float tolerance );
   template void DrawEvaluation( NurbSurface<float> * );
Index: nurbs/hnurbsS.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/hnurbsS.cpp,v
retrieving revision 1.3
diff -u -r1.3 hnurbsS.cpp
--- nurbs/hnurbsS.cpp	17 May 2002 18:24:21 -0000	1.3
+++ nurbs/hnurbsS.cpp	22 Jan 2005 09:18:25 -0000
@@ -103,11 +103,11 @@
   initBase() ;
   offset.resize(baseSurf.ctrlPnts()) ;
 
-  P = baseSurf.ctrlPnts() ;
-  U = baseSurf.knotU() ;
-  V = baseSurf.knotV() ;
-  degU = baseSurf.degreeU() ;
-  degV = baseSurf.degreeV() ;
+  this->P = baseSurf.ctrlPnts() ;
+  this->U = baseSurf.knotU() ;
+  this->V = baseSurf.knotV() ;
+  this->degU = baseSurf.degreeU() ;
+  this->degV = baseSurf.degreeV() ;
 
   //updateSurface() ;
 
@@ -162,11 +162,11 @@
   baseUpdateN = baseLevel_->modifiedN()-1 ; // Set it so that initBase will run
   initBase() ;
   offset.resize(baseSurf.ctrlPnts()) ;
-  P = baseSurf.ctrlPnts() ;
-  U = baseSurf.knotU() ;
-  V = baseSurf.knotV() ;
-  degU = baseSurf.degreeU() ;
-  degV = baseSurf.degreeV() ;
+  this->P = baseSurf.ctrlPnts() ;
+  this->U = baseSurf.knotU() ;
+  this->V = baseSurf.knotV() ;
+  this->degU = baseSurf.degreeU() ;
+  this->degV = baseSurf.degreeV() ;
   //updateSurface() ;
 
 }
@@ -200,7 +200,7 @@
   rU.resize(0) ;
   rV.resize(0) ;
 
-  offset = P ;
+  offset = this->P ;
 }
 
 /*!
@@ -334,11 +334,11 @@
   }
   if(baseLevel_){
     if(initBase()){
-      P = baseSurf.ctrlPnts() ;
-      U = baseSurf.knotU() ;
-      V = baseSurf.knotV() ;
-      degU = baseSurf.degreeU() ;
-      degV = baseSurf.degreeV() ;
+      this->P = baseSurf.ctrlPnts() ;
+      this->U = baseSurf.knotU() ;
+      this->V = baseSurf.knotV() ;
+      this->degU = baseSurf.degreeU() ;
+      this->degV = baseSurf.degreeV() ;
     }
     if(i0>=0 && j0>=0){
       Point_nD<T,N> vecOffset ;
@@ -352,13 +352,13 @@
 	  offset(i0,j0).y()*jvec(i0,j0) +
 	  offset(i0,j0).z()*kvec(i0,j0) ;
       }
-      P(i0,j0).x() = baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
-      P(i0,j0).y() = baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
-      P(i0,j0).z() = baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
+      this->P(i0,j0).x() = baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
+      this->P(i0,j0).y() = baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
+      this->P(i0,j0).z() = baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
     }
     else{
-      for(int i=0;i<P.rows();++i)
-	for(int j=0;j<P.cols();++j){
+      for(int i=0;i<this->P.rows();++i)
+	for(int j=0;j<this->P.cols();++j){
 	  if(offset(i,j).x() != 0 || 
 	     offset(i,j).y() != 0 || offset(i,j).z() || 0){
 	    Point_nD<T,N> vecOffset ;
@@ -372,20 +372,20 @@
 		offset(i,j).y()*jvec(i,j) +
 		offset(i,j).z()*kvec(i,j) ;
 	    }
-	    P(i,j).x() = baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
-	    P(i,j).y() = baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
-	    P(i,j).z() = baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
+	    this->P(i,j).x() = baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
+	    this->P(i,j).y() = baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
+	    this->P(i,j).z() = baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
 	  }
 	}
     }
   }
   else{
     if(i0>=0 && j0>=0)
-      P(i0,j0) = offset(i0,j0) ;
+      this->P(i0,j0) = offset(i0,j0) ;
     else{
-      for(int i=0;i<P.rows();++i)
-	for(int j=0;j<P.cols();++j){
-	  P(i,j) = offset(i,j) ;
+      for(int i=0;i<this->P.rows();++i)
+	for(int j=0;j<this->P.cols();++j){
+	  this->P(i,j) = offset(i,j) ;
 	}
     }
   }
@@ -554,17 +554,17 @@
       return mod ;
   }
 
-  if(u<knotU()[0] || u>knotU()[knotU().n()-1])
+  if(u<this->knotU()[0] || u>this->knotU()[this->knotU().n()-1])
     return -1 ;
-  if(v<knotV()[0] || v>knotU()[knotV().n()-1])
+  if(v<this->knotV()[0] || v>this->knotU()[this->knotV().n()-1])
     return -1 ;
 
   int su = findSpanU(u) ;
   int sv = findSpanV(v) ;
 
-  for(int i=0;i<=degU;++i)
-    for(int j=0;j<=degV;++j){
-      if(offset(su-degU+i,sv+degV+j) != HPoint_nD<T,N>(0,0,0,0))
+  for(int i=0;i<=this->degU;++i)
+    for(int j=0;j<=this->degV;++j){
+      if(offset(su-this->degU+i,sv+this->degV+j) != HPoint_nD<T,N>(0,0,0,0))
 	return level_ ;
     }
 
@@ -742,16 +742,16 @@
 template <class T, int N>
 void HNurbsSurface<T,N>::splitUV(int nu, int nv, Vector<T> &nU, Vector<T> &nV){
 
-  nU.resize(knotU().n()*nu) ;
-  nV.resize(knotV().n()*nv) ;
+  nU.resize(this->knotU().n()*nu) ;
+  nV.resize(this->knotV().n()*nv) ;
   
   int i,j,n ;
 
   n = 0 ; 
-  for(i=1;i<knotU().n();++i){
-    if(knotU()[i] >knotU()[i-1]){
-      T a = knotU()[i-1] ;
-      T b = knotU()[i] ;
+  for(i=1;i<this->knotU().n();++i){
+    if(this->knotU()[i] >this->knotU()[i-1]){
+      T a = this->knotU()[i-1] ;
+      T b = this->knotU()[i] ;
 
 
       for(j=0;j<nu;++j){
@@ -763,10 +763,10 @@
   nU.resize(n) ;
 
   n = 0 ;
-  for(i=1;i<knotV().n();++i){
-    if(knotV()[i] > knotV()[i-1]){
-      T a = knotV()[i-1] ;
-      T b = knotV()[i] ;
+  for(i=1;i<this->knotV().n();++i){
+    if(this->knotV()[i] > this->knotV()[i-1]){
+      T a = this->knotV()[i-1] ;
+      T b = this->knotV()[i] ;
 
       for(j=0;j<nv;++j){
 	nV[n] = a + (b-a)*T(j+1)/T(nv+1) ;
@@ -805,22 +805,22 @@
   int i,j,n ;
 
   if(su<=0)
-    su = degU  ;
+    su = this->degU  ;
   if(sv<=0)
-    sv = degV  ;
-  if(su>degU+1)
-    su = degU+1 ;
-  if(sv>degV+1)
-    sv = degV+1 ;
+    sv = this->degV  ;
+  if(su>this->degU+1)
+    su = this->degU+1 ;
+  if(sv>this->degV+1)
+    sv = this->degV+1 ;
 
-  nU.resize(knotU().n()*nu*su) ;
-  nV.resize(knotV().n()*nv*sv) ;
+  nU.resize(this->knotU().n()*nu*su) ;
+  nV.resize(this->knotV().n()*nv*sv) ;
   
   n = 0 ; 
-  for(i=1;i<knotU().n();++i){
-    if(knotU()[i] >knotU()[i-1]){
-      T a = knotU()[i-1] ;
-      T b = knotU()[i] ;
+  for(i=1;i<this->knotU().n();++i){
+    if(this->knotU()[i] >this->knotU()[i-1]){
+      T a = this->knotU()[i-1] ;
+      T b = this->knotU()[i] ;
 
 
       for(j=0;j<nu;++j){
@@ -835,10 +835,10 @@
   nU.resize(n) ;
 
   n = 0 ;
-  for(i=1;i<knotV().n();++i){
-    if(knotV()[i] > knotV()[i-1]){
-      T a = knotV()[i-1] ;
-      T b = knotV()[i] ;
+  for(i=1;i<this->knotV().n();++i){
+    if(this->knotV()[i] > this->knotV()[i-1]){
+      T a = this->knotV()[i-1] ;
+      T b = this->knotV()[i] ;
 
       for(j=0;j<nv;++j){
 	T v = a + (b-a)*T(j+1)/T(nv+1) ;
@@ -1014,10 +1014,10 @@
     if(!fin.read((char*)&du,sizeof(int))) { delete []type ; return 0 ;}
     if(!fin.read((char*)&dv,sizeof(int))) { delete []type ; return 0 ;}
     
-    resize(nu,nv,du,dv) ;
+    this->resize(nu,nv,du,dv) ;
     
-    if(!fin.read((char*)U.memory(),sizeof(T)*U.n())) { delete []type ; return 0 ;}
-    if(!fin.read((char*)V.memory(),sizeof(T)*V.n())) { delete []type ; return 0 ;}
+    if(!fin.read((char*)this->U.memory(),sizeof(T)*this->U.n())) { delete []type ; return 0 ;}
+    if(!fin.read((char*)this->V.memory(),sizeof(T)*this->V.n())) { delete []type ; return 0 ;}
     
     if(!r1){
       p = new T[3*nu*nv] ;
@@ -1025,10 +1025,10 @@
       p2 = p ;
       for(int i=0;i<nu;i++)
 	for(int j=0;j<nv;j++){
-	  P(i,j).x() = *(p++) ;
-	  P(i,j).y() = *(p++) ;
-	  P(i,j).z() = *(p++) ;
-	  P(i,j).w() = 1.0 ;
+	  this->P(i,j).x() = *(p++) ;
+	  this->P(i,j).y() = *(p++) ;
+	  this->P(i,j).z() = *(p++) ;
+	  this->P(i,j).w() = 1.0 ;
 	}
       delete []p2 ;
     }
@@ -1038,14 +1038,14 @@
       p2 = p ;
       for(int i=0;i<nu;i++)
 	for(int j=0;j<nv;j++){
-	  P(i,j).x() = *(p++) ;
-	  P(i,j).y() = *(p++) ;
-	  P(i,j).z() = *(p++) ;
-	  P(i,j).w() = *(p++) ;
+	  this->P(i,j).x() = *(p++) ;
+	  this->P(i,j).y() = *(p++) ;
+	  this->P(i,j).z() = *(p++) ;
+	  this->P(i,j).w() = *(p++) ;
 	}
       delete []p2 ;
     }
-    offset = P ;
+    offset = this->P ;
     this->updateSurface() ;
   }
   else { // reading the offset information
@@ -1144,29 +1144,29 @@
   if(!fout)
     return 0 ;
   if(!baseLevel_){
-    int prows = P.rows();
-    int pcols = P.cols();
+    int prows = this->P.rows();
+    int pcols = this->P.cols();
     char st = '0' + sizeof(T) ; 
     if(!fout.write((char*)&"hns4",sizeof(char)*4)) return 0 ;
     if(!fout.write((char*)&st,sizeof(char))) return 0 ; 
     if(!fout.write((char*)&prows,sizeof(int))) return 0 ;
     if(!fout.write((char*)&pcols,sizeof(int))) return 0 ;
-    if(!fout.write((char*)&degU,sizeof(int))) return 0 ;
-    if(!fout.write((char*)&degV,sizeof(int))) return 0 ;
-    if(!fout.write((char*)U.memory(),sizeof(T)*U.n())) return 0 ;
-    if(!fout.write((char*)V.memory(),sizeof(T)*V.n())) return 0 ;
+    if(!fout.write((char*)&this->degU,sizeof(int))) return 0 ;
+    if(!fout.write((char*)&this->degV,sizeof(int))) return 0 ;
+    if(!fout.write((char*)this->U.memory(),sizeof(T)*this->U.n())) return 0 ;
+    if(!fout.write((char*)this->V.memory(),sizeof(T)*this->V.n())) return 0 ;
     
     T *p,*p2 ;
-    p = new T[P.rows()*P.cols()*4] ;
+    p = new T[this->P.rows()*this->P.cols()*4] ;
     p2 = p ;
-    for(int i=0;i<P.rows();i++) 
-      for(int j=0;j<P.cols();j++){
+    for(int i=0;i<this->P.rows();i++) 
+      for(int j=0;j<this->P.cols();j++){
 	*p = offset(i,j).x() ; p++ ;
 	*p = offset(i,j).y() ; p++ ;
 	*p = offset(i,j).z() ; p++ ;
 	*p = offset(i,j).w() ; p++ ;
       }
-    if(!fout.write((char*)p2,sizeof(T)*P.rows()*P.cols()*4)) return 0 ;
+    if(!fout.write((char*)p2,sizeof(T)*this->P.rows()*this->P.cols()*4)) return 0 ;
     delete []p2 ;
   }
   else{
@@ -1282,7 +1282,7 @@
   int i,j ;
   j = 0 ;
   for(i=0;i<X.n();++i){
-    if(X[i]>=U[0] && X[i]<=U[U.n()-1]){
+    if(X[i]>=this->U[0] && X[i]<=this->U[this->U.n()-1]){
       Xu[j] = X[i] ;
       ++j ;
     }
@@ -1294,7 +1294,7 @@
       nextLevel_->refineKnotU(Xu) ;
     }
     
-    NurbsSurface<T,N> osurf(degU,degV,U,V,offset) ;
+    NurbsSurface<T,N> osurf(this->degU,this->degV,this->U,this->V,offset) ;
     
     osurf.refineKnotU(Xu) ;
     
@@ -1324,7 +1324,7 @@
   int i,j ;
   j = 0 ;
   for(i=0;i<X.n();++i){
-    if(X[i]>=V[0] && X[i]<=V[V.n()-1]){
+    if(X[i]>=this->V[0] && X[i]<=this->V[this->V.n()-1]){
       Xv[j] = X[i] ;
       ++j ;
     }
@@ -1336,7 +1336,7 @@
       nextLevel_->refineKnotV(Xv) ;
     }
     
-    NurbsSurface<T,N> osurf(degU,degV,U,V,offset) ;
+    NurbsSurface<T,N> osurf(this->degU,this->degV,this->U,this->V,offset) ;
     
     osurf.refineKnotV(Xv) ;
     
@@ -1370,26 +1370,26 @@
 */
 template <class T, int N>
 int HNurbsSurface<T,N>::movePointOffset(T u, T v, const Point_nD<T,N>& delta){
-  P = offset ; 
+  this->P = offset ; 
 
   // by definition the offset has w = 0 , but this isn't valid for
   // the control points, increasing the w by 1, will generate a valid surface
   if(baseLevel_)
-    for(int i=0;i<P.rows();++i)
-      for(int j=0;j<P.cols();++j){
-	P(i,j).w() += T(1) ; 
+    for(int i=0;i<this->P.rows();++i)
+      for(int j=0;j<this->P.cols();++j){
+	this->P(i,j).w() += T(1) ; 
       }
 
   if(NurbsSurface<T,N>::movePoint(u,v,delta)){
-    offset = P ;
+    offset = this->P ;
     // need to reset the offset weights
     if(baseLevel_)
-      for(int i=0;i<P.rows();++i)
-	for(int j=0;j<P.cols();++j){
-	  P(i,j).w() -= T(1) ; 
+      for(int i=0;i<this->P.rows();++i)
+	for(int j=0;j<this->P.cols();++j){
+	  this->P(i,j).w() -= T(1) ; 
       }
     
-    P = baseSurf.ctrlPnts() ; 
+    this->P = baseSurf.ctrlPnts() ; 
     updateSurface() ; 
     return 1 ;
   }
Index: nurbs/hnurbsS_sp.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/hnurbsS_sp.cpp,v
retrieving revision 1.2
diff -u -r1.2 hnurbsS_sp.cpp
--- nurbs/hnurbsS_sp.cpp	13 May 2002 21:07:46 -0000	1.2
+++ nurbs/hnurbsS_sp.cpp	22 Jan 2005 09:18:25 -0000
@@ -43,7 +43,7 @@
 */
 template <class T, int N>
 void HNurbsSurfaceSP<T,N>::updateMaxU() {
-  if(degU>3){
+  if(this->degU>3){
 #ifdef USE_EXCEPTION
     throw NurbsError();
 #else
@@ -53,12 +53,12 @@
 #endif
   }
   else{
-    maxU.resize(P.rows()) ;
-    maxAtU_.resize(P.rows()) ;
-    for(int i=0;i<P.rows();++i){
-      if(!maxInfluence(i,U,degU,maxAtU_[i]))
+    maxU.resize(this->P.rows()) ;
+    maxAtU_.resize(this->P.rows()) ;
+    for(int i=0;i<this->P.rows();++i){
+      if(!maxInfluence(i,this->U,this->degU,maxAtU_[i]))
 	cerr << "Problem in maxInfluence U!\n" ;
-      maxU[i] = nurbsBasisFun(maxAtU_[i],i,degU,U) ;
+      maxU[i] = nurbsBasisFun(maxAtU_[i],i,this->degU,this->U) ;
     }
     
   }
@@ -78,7 +78,7 @@
 */
 template <class T, int N>
 void HNurbsSurfaceSP<T,N>::updateMaxV() {
-  if(degV>3){
+  if(this->degV>3){
 #ifdef USE_EXCEPTION
     throw NurbsError();
 #else
@@ -88,12 +88,12 @@
 #endif
   }
   else{
-    maxV.resize(P.cols()) ;
-    maxAtV_.resize(P.cols()) ;
-    for(int i=0;i<P.cols();++i){
-      if(!maxInfluence(i,V,degV,maxAtV_[i]))
+    maxV.resize(this->P.cols()) ;
+    maxAtV_.resize(this->P.cols()) ;
+    for(int i=0;i<this->P.cols();++i){
+      if(!maxInfluence(i,this->V,this->degV,maxAtV_[i]))
 	cerr << "Problem in maxInfluence V!\n" ;
-      maxV[i] = nurbsBasisFun(maxAtV_[i],i,degV,V) ;
+      maxV[i] = nurbsBasisFun(maxAtV_[i],i,this->degV,this->V) ;
     }
     
   }
@@ -113,18 +113,18 @@
 */
 template <class T, int N>
 void HNurbsSurfaceSP<T,N>::modSurfCPby(int i, int j, const HPoint_nD<T,N>& a) {
-  offset(i,j) +=  a / (maxU[i]*maxV[j]) ; 
-  if(baseLevel_){
+  this->offset(i,j) +=  a / (maxU[i]*maxV[j]) ; 
+  if(this->baseLevel_){
     Point_nD<T,N> vecOffset ; 
-    vecOffset = offset(i,j).x()*ivec(i,j) +
-      offset(i,j).y()*jvec(i,j) +
-      offset(i,j).z()*kvec(i,j) ;
-    P(i,j).x() = baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
-    P(i,j).y() = baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
-    P(i,j).z() = baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
+    vecOffset = this->offset(i,j).x()*this->ivec(i,j) +
+      this->offset(i,j).y()*this->jvec(i,j) +
+      this->offset(i,j).z()*this->kvec(i,j) ;
+    this->P(i,j).x() = this->baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
+    this->P(i,j).y() = this->baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
+    this->P(i,j).z() = this->baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
   }
   else
-    P(i,j) = offset(i,j) ; 
+    this->P(i,j) = this->offset(i,j) ; 
 }
 
 /*!
@@ -151,24 +151,24 @@
 void HNurbsSurfaceSP<T,N>::modOnlySurfCPby(int i, int j, const HPoint_nD<T,N>& a){
   int k ; 
 
-  P = offset ; 
+  this->P = this->offset ; 
 
   // by definition the offset has w = 0 , but this isn't valid for
   // the control points, increasing the w by 1, will generate a valid surface
-  if(baseLevel_)
-    for(k=0;k<P.rows();++k)
-      for(int l=0;l<P.cols();++l)
-	P(k,l).w() += T(1) ; 
+  if(this->baseLevel_)
+    for(k=0;k<this->P.rows();++k)
+      for(int l=0;l<this->P.cols();++l)
+	this->P(k,l).w() += T(1) ; 
 
   int sizeU, sizeV ;
 
-  sizeU = 2*degU+3 ; 
-  if(i-degU-1<0) sizeU += i-degU-1 ; 
-  if(i+degU+1>=P.rows()) sizeU -= i+degU+1-P.rows() ;
-
-  sizeV = 2*degV+3 ;
-  if(j-degV-1<0) sizeV += j-degV-1 ; 
-  if(j+degV+1>=P.cols()) sizeV -= j+degV+1-P.cols() ;
+  sizeU = 2*this->degU+3 ; 
+  if(i-this->degU-1<0) sizeU += i-this->degU-1 ; 
+  if(i+this->degU+1>=this->P.rows()) sizeU -= i+this->degU+1-this->P.rows() ;
+
+  sizeV = 2*this->degV+3 ;
+  if(j-this->degV-1<0) sizeV += j-this->degV-1 ; 
+  if(j+this->degV+1>=this->P.cols()) sizeV -= j+this->degV+1-this->P.cols() ;
   
   Vector<T> u(sizeU) ;
   Vector<T> v(sizeV) ;
@@ -179,16 +179,16 @@
   int n=0;
   int nu = 0 ;
   int nv = 0 ; 
-  for(k=i-degU-1;k<=i+degU+1;++k){
+  for(k=i-this->degU-1;k<=i+this->degU+1;++k){
     if(k<0)
       continue ;
-    if(k>=P.rows())
+    if(k>=this->P.rows())
       break ; 
     nv = 0 ;
-    for(int l=j-degV-1;l<=j+degV+1;++l){
+    for(int l=j-this->degV-1;l<=j+this->degV+1;++l){
       if(l<0)
 	continue ;
-      if(l>=P.cols())
+      if(l>=this->P.cols())
 	break ; 
       if( k == i && j==l){
 	pts[n].x() = a.x() ; 
@@ -216,12 +216,12 @@
   pv.resize(n) ; 
 
   if(NurbsSurface<T,N>::movePoint(u,v,pts,pu,pv)){
-    offset = P ; 
+    this->offset = this->P ; 
     // an offset shouldn't have a weight value.
-    if(baseLevel_)
-      for(k=0;k<P.rows();++k)
-	for(int l=0;l<P.cols();++l)
-	  offset(k,l).w() -= T(1) ; 
+    if(this->baseLevel_)
+      for(k=0;k<this->P.rows();++k)
+	for(int l=0;l<this->P.cols();++l)
+	  this->offset(k,l).w() -= T(1) ; 
   }
   updateSurface(); 
 }
@@ -262,7 +262,7 @@
 HNurbsSurfaceSP<T,N>* HNurbsSurfaceSP<T,N>::addLevel(int n, int s) {
   HNurbsSurfaceSP<T,N> *newLevel ;
 
-  if(nextLevel_)
+  if(this->nextLevel_)
     return 0 ;
 
   Vector<T> newU,newV ;
@@ -289,7 +289,7 @@
 HNurbsSurfaceSP<T,N>* HNurbsSurfaceSP<T,N>::addLevel() {
   HNurbsSurfaceSP<T,N> *newLevel ;
 
-  if(nextLevel_)
+  if(this->nextLevel_)
     return 0 ;
 
   newLevel = new HNurbsSurfaceSP<T,N>(this) ;
@@ -311,23 +311,23 @@
   levelP = nS.nextLevel() ;
 
   NurbsSurface<T,N>::operator=(nS) ;
-  rU = nS.rU ;
-  rV = nS.rV ;
-  offset = nS.offset ;
+  this->rU = nS.rU ;
+  this->rV = nS.rV ;
+  this->offset = nS.offset ;
 
   updateMaxUV() ; 
 
-  firstLevel_ = this ;
+  this->firstLevel_ = this ;
 
   if(levelP){
     HNurbsSurfaceSP<T,N> *newLevel ;
     newLevel =  new HNurbsSurfaceSP<T,N>(this) ; 
     newLevel->copy(*levelP) ;
-    nextLevel_ = newLevel ;
-    lastLevel_ = nextLevel_->lastLevel() ;
+    this->nextLevel_ = newLevel ;
+    this->lastLevel_ = this->nextLevel_->lastLevel() ;
   }
   else{
-    lastLevel_ = this ;
+    this->lastLevel_ = this ;
   }
 
 }
@@ -349,55 +349,55 @@
 template <class T, int N>
 void HNurbsSurfaceSP<T,N>::updateSurface(int i0, int j0){
   if(i0>=0 && j0>=0){
-    if(offset(i0,j0).x()==0.0 && offset(i0,j0).y()==0.0 && offset(i0,j0).z()==0.0)
+    if(this->offset(i0,j0).x()==0.0 && this->offset(i0,j0).y()==0.0 && this->offset(i0,j0).z()==0.0)
       return ;
   }
-  if(baseLevel_){
-    if(initBase()){
-      P = baseSurf.ctrlPnts() ;
-      U = baseSurf.knotU() ;
-      V = baseSurf.knotV() ;
-      degU = baseSurf.degreeU() ;
-      degV = baseSurf.degreeV() ;
+  if(this->baseLevel_){
+    if(this->initBase()){
+      this->P = this->baseSurf.ctrlPnts() ;
+      this->U = this->baseSurf.knotU() ;
+      this->V = this->baseSurf.knotV() ;
+      this->degU = this->baseSurf.degreeU() ;
+      this->degV = this->baseSurf.degreeV() ;
       updateMaxUV() ; 
     }
     if(i0>=0 && j0>=0){
       Point_nD<T,N> vecOffset ;
-      vecOffset = offset(i0,j0).x()*ivec(i0,j0) +
-	offset(i0,j0).y()*jvec(i0,j0) +
-	offset(i0,j0).z()*kvec(i0,j0) ;
-      P(i0,j0).x() = baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
-      P(i0,j0).y() = baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
-      P(i0,j0).z() = baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
+      vecOffset = this->offset(i0,j0).x()*this->ivec(i0,j0) +
+	this->offset(i0,j0).y()*this->jvec(i0,j0) +
+	this->offset(i0,j0).z()*this->kvec(i0,j0) ;
+      this->P(i0,j0).x() = this->baseSurf.ctrlPnts()(i0,j0).x()+vecOffset.x() ;
+      this->P(i0,j0).y() = this->baseSurf.ctrlPnts()(i0,j0).y()+vecOffset.y() ;
+      this->P(i0,j0).z() = this->baseSurf.ctrlPnts()(i0,j0).z()+vecOffset.z() ;
     }
     else{
-      for(int i=0;i<P.rows();++i)
-	for(int j=0;j<P.cols();++j){
-	  if(offset(i,j).x() != 0 || 
-	     offset(i,j).y() != 0 || offset(i,j).z() || 0){
+      for(int i=0;i<this->P.rows();++i)
+	for(int j=0;j<this->P.cols();++j){
+	  if(this->offset(i,j).x() != 0 || 
+	     this->offset(i,j).y() != 0 || this->offset(i,j).z() || 0){
 	    Point_nD<T,N> vecOffset ;
-	    vecOffset = offset(i,j).x()*ivec(i,j) +
-	      offset(i,j).y()*jvec(i,j) +
-	      offset(i,j).z()*kvec(i,j) ;
-	    P(i,j).x() = baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
-	    P(i,j).y() = baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
-	    P(i,j).z() = baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
+	    vecOffset = this->offset(i,j).x()*this->ivec(i,j) +
+	      this->offset(i,j).y()*this->jvec(i,j) +
+	      this->offset(i,j).z()*this->kvec(i,j) ;
+	    this->P(i,j).x() = this->baseSurf.ctrlPnts()(i,j).x()+vecOffset.x() ;
+	    this->P(i,j).y() = this->baseSurf.ctrlPnts()(i,j).y()+vecOffset.y() ;
+	    this->P(i,j).z() = this->baseSurf.ctrlPnts()(i,j).z()+vecOffset.z() ;
 	  }
 	}
     }
   }
   else{
     if(i0>=0 && j0>=0)
-      P(i0,j0) = offset(i0,j0) ;
+      this->P(i0,j0) = this->offset(i0,j0) ;
     else{
-      for(int i=0;i<P.rows();++i)
-	for(int j=0;j<P.cols();++j){
-	  P(i,j) = offset(i,j) ;
+      for(int i=0;i<this->P.rows();++i)
+	for(int j=0;j<this->P.cols();++j){
+	  this->P(i,j) = this->offset(i,j) ;
 	}
     }
   }
 
-  ++updateN ;
+  ++this->updateN ;
 }
 
 /*!
@@ -413,7 +413,7 @@
   if(!okMax())
     updateMaxUV() ; 
   if(upLevel>=0){
-    if(level()<=upLevel){
+    if(this->level()<=upLevel){
       this->updateSurface() ;
     }
   }
@@ -421,9 +421,9 @@
     this->updateSurface() ;
   }
 
-  if(upLevel>level() || upLevel<0){
-    if(nextLevel_)
-      ((HNurbsSurfaceSP<T,N>*)nextLevel_)->updateLevels(upLevel) ;
+  if(upLevel>this->level() || upLevel<0){
+    if(this->nextLevel_)
+      ((HNurbsSurfaceSP<T,N>*)this->nextLevel_)->updateLevels(upLevel) ;
   }
 }
 
@@ -458,10 +458,10 @@
     if(!fin.read((char*)&du,sizeof(int))) { delete []type ; return 0 ;}
     if(!fin.read((char*)&dv,sizeof(int))) { delete []type ; return 0 ;}
     
-    resize(nu,nv,du,dv) ;
+    this->resize(nu,nv,du,dv) ;
     
-    if(!fin.read((char*)U.memory(),sizeof(T)*U.n())) { delete []type ; return 0 ;}
-    if(!fin.read((char*)V.memory(),sizeof(T)*V.n())) { delete []type ; return 0 ;}
+    if(!fin.read((char*)this->U.memory(),sizeof(T)*this->U.n())) { delete []type ; return 0 ;}
+    if(!fin.read((char*)this->V.memory(),sizeof(T)*this->V.n())) { delete []type ; return 0 ;}
     
     if(!r1){
       p = new T[3*nu*nv] ;
@@ -469,10 +469,10 @@
       p2 = p ;
       for(int i=0;i<nu;i++)
 	for(int j=0;j<nv;j++){
-	  P(i,j).x() = *(p++) ;
-	  P(i,j).y() = *(p++) ;
-	  P(i,j).z() = *(p++) ;
-	  P(i,j).w() = 1.0 ;
+	  this->P(i,j).x() = *(p++) ;
+	  this->P(i,j).y() = *(p++) ;
+	  this->P(i,j).z() = *(p++) ;
+	  this->P(i,j).w() = 1.0 ;
 	}
       delete []p2 ;
     }
@@ -482,26 +482,26 @@
       p2 = p ;
       for(int i=0;i<nu;i++)
 	for(int j=0;j<nv;j++){
-	  P(i,j).x() = *(p++) ;
-	  P(i,j).y() = *(p++) ;
-	  P(i,j).z() = *(p++) ;
-	  P(i,j).w() = *(p++) ;
+	  this->P(i,j).x() = *(p++) ;
+	  this->P(i,j).y() = *(p++) ;
+	  this->P(i,j).z() = *(p++) ;
+	  this->P(i,j).w() = *(p++) ;
 	}
       delete []p2 ;
     }
-    offset = P ;
+    this->offset = this->P ;
     this->updateSurface() ;
   }
   else { // reading the offset information
     int ru,rv ;
     if(!fin.read((char*)&ru,sizeof(int))) { delete []type ; return 0 ;}
     if(!fin.read((char*)&rv,sizeof(int))) { delete []type ; return 0 ;}
-    rU.resize(ru) ;
-    rV.resize(rv) ;
-    if(rU.n()>0)
-      if(!fin.read((char*)rU.memory(),sizeof(T)*rU.n())) { delete []type ; return 0 ;}
-    if(rV.n()>0)
-      if(!fin.read((char*)rV.memory(),sizeof(T)*rV.n())) { delete []type ; return 0 ;}
+    this->rU.resize(ru) ;
+    this->rV.resize(rv) ;
+    if(this->rU.n()>0)
+      if(!fin.read((char*)this->rU.memory(),sizeof(T)*this->rU.n())) { delete []type ; return 0 ;}
+    if(this->rV.n()>0)
+      if(!fin.read((char*)this->rV.memory(),sizeof(T)*this->rV.n())) { delete []type ; return 0 ;}
     
     if(!fin.read((char*)&nu,sizeof(int))) { delete []type ; return 0 ;}
     if(!fin.read((char*)&nv,sizeof(int))) { delete []type ; return 0 ;}
@@ -509,16 +509,16 @@
     p = new T[4*nu*nv] ;
     if(!fin.read((char*)p,sizeof(T)*4*nu*nv)) { delete []type ; return 0 ;}
     p2 = p ;
-    offset.resize(nu,nv) ;
+    this->offset.resize(nu,nv) ;
     for(int i=0;i<nu;i++)
       for(int j=0;j<nv;j++){
-	offset(i,j).x() = *(p++) ;
-	offset(i,j).y() = *(p++) ;
-	offset(i,j).z() = *(p++) ;
-	offset(i,j).w() = *(p++) ;
+	this->offset(i,j).x() = *(p++) ;
+	this->offset(i,j).y() = *(p++) ;
+	this->offset(i,j).z() = *(p++) ;
+	this->offset(i,j).w() = *(p++) ;
       }
     delete []p2 ;    
-    --baseUpdateN ;
+    --this->baseUpdateN ;
     this->updateSurface() ;
   }
 
Index: nurbs/matrixRT.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/matrixRT.cpp,v
retrieving revision 1.3
diff -u -r1.3 matrixRT.cpp
--- nurbs/matrixRT.cpp	13 Jan 2003 19:41:35 -0000	1.3
+++ nurbs/matrixRT.cpp	22 Jan 2005 09:18:25 -0000
@@ -55,13 +55,13 @@
   // *this = C.translate(x,y,z)*B.rotate(ax,ay,az) ;
   rotate(ax,ay,az) ;
 #ifdef COLUMN_ORDER
-  m[12] = x ;
-  m[13] = y ;
-  m[14] = z ;  
+  this->m[12] = x ;
+  this->m[13] = y ;
+  this->m[14] = z ;  
 #else
-  m[3] = x ;
-  m[7] = y ;
-  m[11] = z ;
+  this->m[3] = x ;
+  this->m[7] = y ;
+  this->m[11] = z ;
 #endif
 }
 
@@ -75,8 +75,8 @@
  */
 template <class T>
 MatrixRT<T>::MatrixRT() : Matrix<T>(4,4) {
-  reset(0) ;
-  diag(1.0) ;
+  this->reset(0) ;
+  this->diag(1.0) ;
 }
 
 /*!
@@ -148,33 +148,33 @@
   t10 = cos(ax);
   t13 = t4*t6;
 #ifdef COLUMN_ORDER
-  m[0] = t1*t2;
-  m[4] = -t4*t2;
-  m[8] = t6;
-  m[12] = 0 ;
-  m[1] = t7*t8+t4*t10;
-  m[5] = -t13*t8+t1*t10;
-  m[9] = -t2*t8;
-  m[13] = 0 ;
-  m[2] = -t7*t10+t4*t8;
-  m[6] = t13*t10+t1*t8;
-  m[10] = t2*t10;
-  m[14] = m[3] = m[7] = m[11] = 0.0 ;
-  m[15] = 1.0 ;
+  this->m[0] = t1*t2;
+  this->m[4] = -t4*t2;
+  this->m[8] = t6;
+  this->m[12] = 0 ;
+  this->m[1] = t7*t8+t4*t10;
+  this->m[5] = -t13*t8+t1*t10;
+  this->m[9] = -t2*t8;
+  this->m[13] = 0 ;
+  this->m[2] = -t7*t10+t4*t8;
+  this->m[6] = t13*t10+t1*t8;
+  this->m[10] = t2*t10;
+  this->m[14] = this->m[3] = this->m[7] = this->m[11] = 0.0 ;
+  this->m[15] = 1.0 ;
 #else
-  m[0] = t1*t2;
-  m[1] = -t4*t2;
-  m[2] = t6;
-  m[3] = 0 ;
-  m[4] = t7*t8+t4*t10;
-  m[5] = -t13*t8+t1*t10;
-  m[6] = -t2*t8;
-  m[7] = 0 ;
-  m[8] = -t7*t10+t4*t8;
-  m[9] = t13*t10+t1*t8;
-  m[10] = t2*t10;
-  m[11] = m[12] = m[13] = m[14] = 0 ;
-  m[15] = 1.0 ;
+  this->m[0] = t1*t2;
+  this->m[1] = -t4*t2;
+  this->m[2] = t6;
+  this->m[3] = 0 ;
+  this->m[4] = t7*t8+t4*t10;
+  this->m[5] = -t13*t8+t1*t10;
+  this->m[6] = -t2*t8;
+  this->m[7] = 0 ;
+  this->m[8] = -t7*t10+t4*t8;
+  this->m[9] = t13*t10+t1*t8;
+  this->m[10] = t2*t10;
+  this->m[11] = this->m[12] = this->m[13] = this->m[14] = 0 ;
+  this->m[15] = 1.0 ;
 #endif
   return *this ;
 }
@@ -207,33 +207,33 @@
   t9 = (T)sin((double)ax);
   t17 = t4*t7;
 #ifdef COLUMN_ORDER
-  m[0] = t1*t2;
-  m[4] = -t4*t5+t8*t9;
-  m[8] = t4*t9+t8*t5;
-  m[12] = 0.0 ;
-  m[1] = t4*t2;
-  m[5] = t1*t5+t17*t9;
-  m[9] = -t1*t9+t17*t5;
-  m[13] = 0.0 ;
-  m[2] = -t7;
-  m[6] = t2*t9;
-  m[10] = t2*t5;
-  m[14] = m[3] = m[7] = m[11] = 0 ;
-  m[15] = 1.0 ;
+  this->m[0] = t1*t2;
+  this->m[4] = -t4*t5+t8*t9;
+  this->m[8] = t4*t9+t8*t5;
+  this->m[12] = 0.0 ;
+  this->m[1] = t4*t2;
+  this->m[5] = t1*t5+t17*t9;
+  this->m[9] = -t1*t9+t17*t5;
+  this->m[13] = 0.0 ;
+  this->m[2] = -t7;
+  this->m[6] = t2*t9;
+  this->m[10] = t2*t5;
+  this->m[14] = this->m[3] = this->m[7] = this->m[11] = 0 ;
+  this->m[15] = 1.0 ;
 #else
-  m[0] = t1*t2;
-  m[1] = -t4*t5+t8*t9;
-  m[2] = t4*t9+t8*t5;
-  m[3] = 0.0 ;
-  m[4] = t4*t2;
-  m[5] = t1*t5+t17*t9;
-  m[6] = -t1*t9+t17*t5;
-  m[7] = 0.0 ;
-  m[8] = -t7;
-  m[9] = t2*t9;
-  m[10] = t2*t5;
-  m[11] = m[12] = m[13] = m[14] = 0 ;
-  m[15] = 1.0 ;
+  this->m[0] = t1*t2;
+  this->m[1] = -t4*t5+t8*t9;
+  this->m[2] = t4*t9+t8*t5;
+  this->m[3] = 0.0 ;
+  this->m[4] = t4*t2;
+  this->m[5] = t1*t5+t17*t9;
+  this->m[6] = -t1*t9+t17*t5;
+  this->m[7] = 0.0 ;
+  this->m[8] = -t7;
+  this->m[9] = t2*t9;
+  this->m[10] = t2*t5;
+  this->m[11] = this->m[12] = this->m[13] = this->m[14] = 0 ;
+  this->m[15] = 1.0 ;
 #endif
   return *this ;
 }
@@ -249,16 +249,16 @@
  */
 template <class T>
 MatrixRT<T>& MatrixRT<T>::translate(T x, T y, T z){
-  reset(0) ;
-  diag(1.0) ;
+  this->reset(0) ;
+  this->diag(1.0) ;
 #ifdef COLUMN_ORDER
-  m[12] = x ;
-  m[13] = y ;
-  m[14] = z ;
+  this->m[12] = x ;
+  this->m[13] = y ;
+  this->m[14] = z ;
 #else
-  m[3] = x ;
-  m[7] = y ;
-  m[11] = z ;
+  this->m[3] = x ;
+  this->m[7] = y ;
+  this->m[11] = z ;
 #endif
   return *this ;
 }
@@ -275,11 +275,11 @@
  */
 template <class T>
 MatrixRT<T>& MatrixRT<T>::scale(T x, T y, T z){
-  reset(0) ;
-  m[0] = x ;
-  m[5] = y ;
-  m[10] = z ;
-  m[15] = 1.0 ;
+  this->reset(0) ;
+  this->m[0] = x ;
+  this->m[5] = y ;
+  this->m[10] = z ;
+  this->m[15] = 1.0 ;
   return *this ;
 }
 
@@ -420,7 +420,7 @@
     error.fatal() ;
   }
   T *a,*b ;
-  a = m-1 ;
+  a = this->m-1 ;
   b = M[0] - 1 ;
   for(int i=0;i<16;++i){
     *(++a) = *(++b) ;
@@ -439,7 +439,7 @@
 template <class T>
 MatrixRT<T>& MatrixRT<T>::operator=(const MatrixRT<T>& M) {
   T *a,*b ;
-  a = m-1 ;
+  a = this->m-1 ;
   b = M.m - 1 ;
   for(int i=0;i<16;++i){
     *(++a) = *(++b) ;
Index: nurbs/nurbs.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbs.cpp,v
retrieving revision 1.5
diff -u -r1.5 nurbs.cpp
--- nurbs/nurbs.cpp	27 Jan 2003 11:37:36 -0000	1.5
+++ nurbs/nurbs.cpp	22 Jan 2005 09:18:25 -0000
@@ -394,9 +394,9 @@
   T du,dv ;
   // compute a coarse distance for the curve
   Point_nD<T,N> a,b,c ;
-  a = pointAt(0.0) ;
-  b = pointAt(0.5) ;
-  c = pointAt(1.0) ;
+  a = this->pointAt(0.0) ;
+  b = this->pointAt(0.5) ;
+  c = this->pointAt(1.0) ;
 
   T distance = norm(b-a) + norm(c-b) ;
 
@@ -5224,7 +5224,7 @@
 */
 template <class T, int N>
 BasicList<Point_nD<T,N> > NurbsCurve<T,N>::tesselate(T tolerance,BasicList<T> *uk) const {
-  BasicList<Point_nD<T,N> > list,list2 ;
+  BasicList<Point_nD<T,N> > list, list2 ;
 
   NurbsCurveArray<T,N> ca ;
   decompose(ca) ;
@@ -5271,7 +5271,7 @@
   }
   else{
     for(int i=0;i<ca.n();++i){
-      list2 = ca[i].tesselate(tolerance,uk) ;
+      list2 = (const BasicList<Point_nD<T,N> >&) ca[i].tesselate(tolerance,uk) ;
 
       // remove the last point from the list to elliminate
       list.erase((BasicNode<Point_nD<T,N> >*)list.last()) ;
Index: nurbs/nurbsS.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbsS.cpp,v
retrieving revision 1.4
diff -u -r1.4 nurbsS.cpp
--- nurbs/nurbsS.cpp	13 Jan 2003 19:42:01 -0000	1.4
+++ nurbs/nurbsS.cpp	22 Jan 2005 09:18:25 -0000
@@ -3795,12 +3795,12 @@
   // we use and angle of 36 to view the object
   // and position the rest according to this.
   Point_nD<T,N> minP, maxP ;
-  minP.x() = extremum(1,coordX) ;
-  minP.y() = extremum(1,coordY) ;
-  minP.z() = extremum(1,coordZ) ;
-  maxP.x() = extremum(0,coordX) ;
-  maxP.y() = extremum(0,coordY) ;
-  maxP.z() = extremum(0,coordZ) ;
+  minP.x() = this->extremum(1,coordX) ;
+  minP.y() = this->extremum(1,coordY) ;
+  minP.z() = this->extremum(1,coordZ) ;
+  maxP.x() = this->extremum(0,coordX) ;
+  maxP.y() = this->extremum(0,coordY) ;
+  maxP.z() = this->extremum(0,coordZ) ;
 
   Point_nD<T,N> lookAt  ;
   lookAt.x() = (minP.x()+maxP.x())/2.0 ;
@@ -3893,12 +3893,12 @@
   // we use and angle of 36 to view the object
   // and position the rest according to this.
   Point_nD<T,N> minP, maxP ;
-  minP.x() = extremum(1,coordX) ;
-  minP.y() = extremum(1,coordY) ;
-  minP.z() = extremum(1,coordZ) ;
-  maxP.x() = extremum(0,coordX) ;
-  maxP.y() = extremum(0,coordY) ;
-  maxP.z() = extremum(0,coordZ) ;
+  minP.x() = this->extremum(1,coordX) ;
+  minP.y() = this->extremum(1,coordY) ;
+  minP.z() = this->extremum(1,coordZ) ;
+  maxP.x() = this->extremum(0,coordX) ;
+  maxP.y() = this->extremum(0,coordY) ;
+  maxP.z() = this->extremum(0,coordZ) ;
 
   Point_nD<T,N> lookAt  ;
   lookAt.x() = (minP.x()+maxP.x())/2.0 ;
@@ -4078,12 +4078,12 @@
   }
 
   Point_nD<T,N> minP, maxP ;
-  minP.x() = extremum(1,coordX) ;
-  minP.y() = extremum(1,coordY) ;
-  minP.z() = extremum(1,coordZ) ;
-  maxP.x() = extremum(0,coordX) ;
-  maxP.y() = extremum(0,coordY) ;
-  maxP.z() = extremum(0,coordZ) ;
+  minP.x() = this->extremum(1,coordX) ;
+  minP.y() = this->extremum(1,coordY) ;
+  minP.z() = this->extremum(1,coordZ) ;
+  maxP.x() = this->extremum(0,coordX) ;
+  maxP.y() = this->extremum(0,coordY) ;
+  maxP.z() = this->extremum(0,coordZ) ;
 
   Point_nD<T,N> lookAt  ;
   lookAt.x() = (minP.x()+maxP.x())/2.0 ;
Index: nurbs/nurbsS_sp.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbsS_sp.cpp,v
retrieving revision 1.2
diff -u -r1.2 nurbsS_sp.cpp
--- nurbs/nurbsS_sp.cpp	13 May 2002 21:07:46 -0000	1.2
+++ nurbs/nurbsS_sp.cpp	22 Jan 2005 09:18:25 -0000
@@ -43,7 +43,7 @@
 */
 template <class T, int N>
 void NurbsSurfaceSP<T,N>::updateMaxU() {
-  if(degU>3){
+  if(this->degU>3){
 #ifdef USE_EXCEPTION
     throw NurbsInputError();
 #else
@@ -53,12 +53,12 @@
 #endif
   }
   else{
-    maxU.resize(P.rows()) ;
-    maxAtU_.resize(P.rows()) ;
-    for(int i=0;i<P.rows();++i){
-      if(!maxInfluence(i,U,degU,maxAtU_[i]))
+    maxU.resize(this->P.rows()) ;
+    maxAtU_.resize(this->P.rows()) ;
+    for(int i=0;i<this->P.rows();++i){
+      if(!maxInfluence(i,this->U,this->degU,maxAtU_[i]))
 	cerr << "Problem in maxInfluence U!\n" ;
-      maxU[i] = nurbsBasisFun(maxAtU_[i],i,degU,U) ;
+      maxU[i] = nurbsBasisFun(maxAtU_[i],i,this->degU,this->U) ;
     }
     
   }
@@ -78,7 +78,7 @@
 */
 template <class T, int N>
 void NurbsSurfaceSP<T,N>::updateMaxV() {
-  if(degV>3){
+  if(this->degV>3){
 #ifdef USE_EXCEPTION
     throw NurbsInputError();
 #else
@@ -88,12 +88,12 @@
 #endif
   }
   else{
-    maxV.resize(P.cols()) ;
-    maxAtV_.resize(P.cols()) ;
-    for(int i=0;i<P.cols();++i){
-      if(!maxInfluence(i,V,degV,maxAtV_[i]))
+    maxV.resize(this->P.cols()) ;
+    maxAtV_.resize(this->P.cols()) ;
+    for(int i=0;i<this->P.cols();++i){
+      if(!maxInfluence(i,this->V,this->degV,maxAtV_[i]))
 	cerr << "Problem in maxInfluence V!\n" ;
-      maxV[i] = nurbsBasisFun(maxAtV_[i],i,degV,V) ;
+      maxV[i] = nurbsBasisFun(maxAtV_[i],i,this->degV,this->V) ;
     }
     
   }
@@ -124,11 +124,11 @@
 NurbsSurfaceSP<T,N> NurbsSurfaceSP<T,N>::generateParallel(T d) const {
   NurbsSurfaceSP<T,N> p(*this) ;
 
-  Vector< Point_nD<T,N> > offset(P.rows()*P.cols()) ;
-  Vector<T> ur(P.rows()*P.cols()) ;
-  Vector<T> vr(P.rows()*P.cols()) ;
-  Vector_INT Du(P.rows()*P.cols()) ;
-  Vector_INT Dv(P.rows()*P.cols()) ;
+  Vector< Point_nD<T,N> > offset(this->P.rows()*this->P.cols()) ;
+  Vector<T> ur(this->P.rows()*this->P.cols()) ;
+  Vector<T> vr(this->P.rows()*this->P.cols()) ;
+  Vector_INT Du(this->P.rows()*this->P.cols()) ;
+  Vector_INT Dv(this->P.rows()*this->P.cols()) ;
 
   Du.reset(0) ;
   Dv.reset(0) ;
@@ -137,8 +137,8 @@
 
   no = 0 ;
 
-  for(i=0;i<P.rows();++i)
-    for(j=0;j<P.cols();++j){
+  for(i=0;i<this->P.rows();++i)
+    for(j=0;j<this->P.cols();++j){
       Point_nD<T,N> norm ;
       norm = normal(maxAtU_[i],maxAtV_[j]) ;
       if(norm.x() == T(0) && 
@@ -155,19 +155,19 @@
 	  norm /= T(2) ;
 	  ok = 1 ;
 	}
-	if(i==P.rows()-1 && j==P.cols()-1){
+	if(i==this->P.rows()-1 && j==this->P.cols()-1){
 	  norm = normal(maxAtU_[i]-delta,maxAtV_[j]) ;
 	  norm += normal(maxAtU_[i],maxAtV_[j]-delta) ;
 	  norm /= T(2) ;
 	  ok = 1 ;
 	}
-	if(i==0 && j==P.cols()-1){
+	if(i==0 && j==this->P.cols()-1){
 	  norm = normal(maxAtU_[i]-delta,maxAtV_[j]) ;
 	  norm += normal(maxAtU_[i],maxAtV_[j]+delta) ;
 	  norm /= T(2) ;
 	  ok = 1 ;
 	}
-	if(i==P.rows()-1 && j==0){
+	if(i==this->P.rows()-1 && j==0){
 	  norm = normal(maxAtU_[i]-delta,maxAtV_[j]) ;
 	  norm += normal(maxAtU_[i],maxAtV_[j]+delta) ;
 	  norm /= T(2) ;
@@ -178,7 +178,7 @@
 	  while(norm.x() == T(0) && 
 	     norm.y() == T(0) &&
 	     norm.z() == T(0)){
-	    if( nt*d >(U[U.n()-1]-U[0])){
+	    if( nt*d >(this->U[this->U.n()-1]-this->U[0])){
 #ifdef USE_EXCEPTION
 	      throw NurbsComputationError();
 #else
@@ -188,12 +188,12 @@
 #endif
 	    }
 	    T u1,u2,v1,v2 ;
-	    if(i==0 || i==P.rows()-1){
+	    if(i==0 || i==this->P.rows()-1){
 	      u1 = u2 = maxAtU_[i] ;
 	      v1 = maxAtV_[j]+ nt*delta ;
 	      v2 = maxAtV_[j]- nt*delta ;
-	      if(v1>V[V.n()-1]) v1 = V[V.n()-1] ;
-	      if(v2<V[0]) v2 = V[0] ;
+	      if(v1>this->V[this->V.n()-1]) v1 = this->V[this->V.n()-1] ;
+	      if(v2<this->V[0]) v2 = this->V[0] ;
 	      norm = normal(u1,v1);
 	      norm += normal(u2,v2) ;
 	      norm /= 2 ; 
@@ -202,8 +202,8 @@
 	      u1 = maxAtU_[i]- nt*delta ;
 	      u2 = maxAtU_[i]+ nt*delta ;
 	      v1 = v2 = maxAtV_[j] ;
-	      if(u1 < U[0]) u1 = U[0] ;
-	      if(u2 > U[U.n()-1]) u2 = U[U.n()-1] ;
+	      if(u1 < this->U[0]) u1 = this->U[0] ;
+	      if(u2 > this->U[this->U.n()-1]) u2 = this->U[this->U.n()-1] ;
 
 	      T u3,v3 ;
 	      u3 = maxAtU_[i] ;
@@ -212,8 +212,8 @@
 	      else
 		v3 = maxAtV_[j]- nt*delta ;
 
-	      if(v3<V[0]) v3 = V[0] ;
-	      if(v3>V[V.n()-1]) v3 = V[V.n()-1] ;
+	      if(v3<this->V[0]) v3 = this->V[0] ;
+	      if(v3>this->V[this->V.n()-1]) v3 = this->V[this->V.n()-1] ;
 
 	      norm = normal(u1,v1);
 	      norm += normal(u2,v2) ;
@@ -263,13 +263,13 @@
 
   int sizeU, sizeV ;
 
-  sizeU = 2*degU+3 ; 
-  if(i-degU-1<0) sizeU += i-degU-1 ; 
-  if(i+degU+1>=P.rows()) sizeU -= i+degU+1-P.rows() ;
-
-  sizeV = 2*degV+3 ;
-  if(j-degV-1<0) sizeV += j-degV-1 ; 
-  if(j+degV+1>=P.cols()) sizeV -= j+degV+1-P.cols() ;
+  sizeU = 2*this->degU+3 ; 
+  if(i-this->degU-1<0) sizeU += i-this->degU-1 ; 
+  if(i+this->degU+1>=this->P.rows()) sizeU -= i+this->degU+1-this->P.rows() ;
+
+  sizeV = 2*this->degV+3 ;
+  if(j-this->degV-1<0) sizeV += j-this->degV-1 ; 
+  if(j+this->degV+1>=this->P.cols()) sizeV -= j+this->degV+1-this->P.cols() ;
   
   Vector<T> u(sizeU) ;
   Vector<T> v(sizeV) ;
@@ -280,16 +280,16 @@
   int n=0;
   int nu = 0 ;
   int nv = 0 ; 
-  for(int k=i-degU-1;k<=i+degU+1;++k){
+  for(int k=i-this->degU-1;k<=i+this->degU+1;++k){
     if(k<0)
       continue ;
-    if(k>=P.rows())
+    if(k>=this->P.rows())
       break ; 
     nv = 0 ;
-    for(int l=j-degV-1;l<=j+degV+1;++l){
+    for(int l=j-this->degV-1;l<=j+this->degV+1;++l){
       if(l<0)
 	continue ;
-      if(l>=P.cols())
+      if(l>=this->P.cols())
 	break ; 
       if( k == i && j==l){
 	pts[n].x() = a.x() ; 
Index: nurbs/nurbsS_sp.h
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbsS_sp.h,v
retrieving revision 1.3
diff -u -r1.3 nurbsS_sp.h
--- nurbs/nurbsS_sp.h	13 Jan 2003 19:42:05 -0000	1.3
+++ nurbs/nurbsS_sp.h	22 Jan 2005 09:18:25 -0000
@@ -79,7 +79,7 @@
 
 
   void modSurfCPby(int i, int j, const HPoint_nD<T,N>& a) //!< Moves a surface point by a value
-    { P(i,j) +=  a / (maxU[i]*maxV[j]) ;  }
+    { this->P(i,j) +=  a / (maxU[i]*maxV[j]) ;  }
   void modSurfCP(int i, int j, const HPoint_nD<T,N>& a) //!< Moves a surface point to a value
     { modSurfCPby(i,j,a-surfP(i,j)) ;  }
 
Index: nurbs/nurbsSub.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbsSub.cpp,v
retrieving revision 1.3
diff -u -r1.3 nurbsSub.cpp
--- nurbs/nurbsSub.cpp	13 Jan 2003 19:42:10 -0000	1.3
+++ nurbs/nurbsSub.cpp	22 Jan 2005 09:18:25 -0000
@@ -908,7 +908,7 @@
   
   /* Allocate storage for the grid of points generated */
   
-  CHECK( pts = new (SurfSample<T>*)[Granularity+1]);
+  CHECK( pts = new SurfSample<T>* [Granularity+1]);
   CHECK( pts[0] = new SurfSample<T>[(Granularity+1)*(Granularity+1)]);
   
   for (i = 1; i <= Granularity; i++)
@@ -987,7 +987,7 @@
   
   if (! *alpha)	/* Must allocate alpha */
     {
-      CHECK( *alpha = new (T*)[k+1]);
+      CHECK( *alpha = new T* [k+1]);
       for (i = 0; i <= k; i++)
 	CHECK( (*alpha)[i] = new T[(m + n + 1)]);
     }
Index: nurbs/nurbs_sp.cpp
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbs_sp.cpp,v
retrieving revision 1.3
diff -u -r1.3 nurbs_sp.cpp
--- nurbs/nurbs_sp.cpp	13 Jan 2003 19:42:25 -0000	1.3
+++ nurbs/nurbs_sp.cpp	22 Jan 2005 09:18:25 -0000
@@ -47,7 +47,7 @@
 */
 template <class T, int N>
 void NurbsCurveSP<T,N>::updateMaxU() {
-  if(deg_>3){
+  if(this->deg_>3){
 #ifdef USE_EXCEPTION
     throw NurbsInputError();
 #else
@@ -57,10 +57,10 @@
 #endif
   }
   else{
-    maxU.resize(P.n()) ;
-    maxAt_.resize(P.n()) ;
-    for(int i=0;i<P.n();++i){
-      if(!maxInfluence(i,U,deg_,maxAt_[i]))
+    maxU.resize(this->P.n()) ;
+    maxAt_.resize(this->P.n()) ;
+    for(int i=0;i<this->P.n();++i){
+      if(!maxInfluence(i,this->U,this->deg_,maxAt_[i]))
 	cerr << "Problem in maxInfluence U!\n" ;
       if(i>0)
 	if(maxAt_[i]<maxAt_[i-1]){
@@ -75,7 +75,7 @@
 	  error.fatal() ; 
 #endif
 	}
-      maxU[i] = basisFun(maxAt_[i],i,deg_) ;
+      maxU[i] = basisFun(maxAt_[i],i,this->deg_) ;
     }
     
   }
@@ -102,14 +102,14 @@
 */
 template <class T, int N>
 void NurbsCurveSP<T,N>::modOnlySurfCPby(int i, const HPoint_nD<T,N>& a){
-  Vector<T> u(2*deg_+3) ;
-  Vector< Point_nD<T,N> > pts(2*deg_+3) ; 
+  Vector<T> u(2*this->deg_+3) ;
+  Vector< Point_nD<T,N> > pts(2*this->deg_+3) ; 
 
   int n=0;
-  for(int j=i-deg_-1;j<=i+deg_+1;++j){
+  for(int j=i-this->deg_-1;j<=i+this->deg_+1;++j){
     if(j<0)
       continue ;
-    if(j>=P.n())
+    if(j>=this->P.n())
       break ; 
     u[n] = maxAt_[j] ;
     if( j == i){
Index: nurbs/nurbs_sp.h
===================================================================
RCS file: /cvsroot/libnurbs/nurbs++/nurbs/nurbs_sp.h,v
retrieving revision 1.3
diff -u -r1.3 nurbs_sp.h
--- nurbs/nurbs_sp.h	17 May 2002 18:24:21 -0000	1.3
+++ nurbs/nurbs_sp.h	22 Jan 2005 09:18:25 -0000
@@ -72,7 +72,7 @@
   int read(ifstream &fin) ;
 
   void modSurfCPby(int i, const HPoint_nD<T,N>& a) 
-    { P[i] +=  a / maxU[i] ;  }
+    { this->P[i] +=  a / maxU[i] ;  }
   void modSurfCP(int i, const HPoint_nD<T,N>& a) 
     { modSurfCPby(i,a-surfP(i)) ;  }
 
