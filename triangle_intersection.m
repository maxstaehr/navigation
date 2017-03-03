function [out hit] = triangle_intersection(V1,...  % Triangle vertices
                             V2,...
                             V3,...
                              O,...  %Ray origin
                               D)  %Ray direction
                               
      out = 0;
      hit = 0;
  
  EPSILON = 0.000001;
  %Find vectors for two edges sharing V1
  e1 = V2-V1;
  e2 = V3-V1;

  
  %Begin calculating determinant - also used to calculate u parameter  
  P = cross(D,e2);
  %if determinant is near zero, ray lies in plane of triangle
  det = dot(e1, P);
  %NOT CULLING
  if(det > -EPSILON && det < EPSILON)
      out= 0;
      hit = 0;
      return;
  end
  
  inv_det = 1/ det;
 
  %calculate distance from V1 to ray origin
  T = O-V1;
  
  %Calculate u parameter and test bound
  u = dot(T, P) * inv_det;
  %The intersection lies outside of the triangle
  if(u < 0 || u > 1)
      out = 0;
      hit = 0;
      return;
  end
 
  %Prepare to test v parameter
  Q = cross(T,e1);
 
  %Calculate V parameter and test bound
  v = dot(D, Q) * inv_det;
  %The intersection lies outside of the triangle
  if(v < 0 || u + v  > 1.0)
      out = 0;
      hit = 0;
      return;
  end
      
 
  t = dot(e2, Q) * inv_det;
 
  if(t > EPSILON)  %ray intersection
    out = t;
    hit = 1;
    return;
  end
end
  