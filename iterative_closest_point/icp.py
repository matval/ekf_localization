# -*- coding: utf-8 -*-
"""
Created on Thu Jan 23 12:43:42 2020

@author: mat_v
"""

'''
Perform the Iterative Closest Point algorithm on three dimensional point clouds.

% [TR, TT] = icp(q,p)   returns the rotation matrix TR and translation
% vector TT that minimizes the distances from (TR * p + TT) to q.
% p is a 3xm matrix and q is a 3xn matrix.
%
% [TR, TT] = icp(q,p,k)   forces the algorithm to make k iterations
% exactly. The default is 10 iterations.
%
% [TR, TT, ER] = icp(q,p,k)   also returns the RMS of errors for k
% iterations in a (k+1)x1 vector. ER(0) is the initial error.
%
% [TR, TT, ER, t] = icp(q,p,k)   also returns the calculation times per
% iteration in a (k+1)x1 vector. t(0) is the time consumed for preprocessing.
%
% Additional settings may be provided in a parameter list:
%
% Boundary
%       {[]} | 1x? vector
%       If EdgeRejection is set, a vector can be provided that indexes into
%       q and specifies which points of q are on the boundary.
%
% EdgeRejection
%       {false} | true
%       If EdgeRejection is true, point matches to edge vertices of q are
%       ignored. Requires that boundary points of q are specified using
%       Boundary or that a triangulation matrix for q is provided.
%
% Extrapolation
%       {false} | true
%       If Extrapolation is true, the iteration direction will be evaluated
%       and extrapolated if possible using the method outlined by 
%       Besl and McKay 1992.
%
% Matching
%       {bruteForce} | Delaunay | kDtree
%       Specifies how point matching should be done. 
%       bruteForce is usually the slowest and kDtree is the fastest.
%       Note that the kDtree option is depends on the Statistics Toolbox
%       v. 7.3 or higher.
%
% Minimize
%       {point} | plane | lmaPoint
%       Defines whether point to point or point to plane minimization
%       should be performed. point is based on the SVD approach and is
%       usually the fastest. plane will often yield higher accuracy. It 
%       uses linearized angles and requires surface normals for all points 
%       in q. Calculation of surface normals requires substantial pre
%       proccessing.
%       The option lmaPoint does point to point minimization using the non
%       linear least squares Levenberg Marquardt algorithm. Results are
%       generally the same as in points, but computation time may differ.
%
% Normals
%       {[]} | n x 3 matrix
%       A matrix of normals for the n points in q might be provided.
%       Normals of q are used for point to plane minimization.
%       Else normals will be found through a PCA of the 4 nearest
%       neighbors.
%
% ReturnAll
%       {false} | true
%       Determines whether R and T should be returned for all iterations
%       or only for the last one. If this option is set to true, R will be
%       a 3x3x(k+1) matrix and T will be a 3x1x(k+1) matrix.
%
% Triangulation
%       {[]} | ? x 3 matrix
%       A triangulation matrix for the points in q can be provided,
%       enabling EdgeRejection. The elements should index into q, defining
%       point triples that act together as triangles.
%
% Verbose
%       {false} | true
%       Enables extrapolation output in the Command Window.
%
% Weight
%       {@(match)ones(1,m)} | Function handle
%       For point or plane minimization, a function handle to a weighting 
%       function can be provided. The weighting function will be called 
%       with one argument, a 1xm vector that specifies point pairs by 
%       indexing into q. The weighting function should return a 1xm vector 
%       of weights for every point pair.
%
% WorstRejection
%       {0} | scalar in ]0; 1[
%       Reject a given percentage of the worst point pairs, based on their
%       Euclidean distance.
%
% Martin Kjer and Jakob Wilm, Technical University of Denmark, 2012
% Use the inputParser class to validate input arguments.
'''

def icp(q, p, n_iter, Matching = "kDtree", tolerance = 0.01):
    
    dist_max = 10.0
    
    new_q = np.array([[],[]])
    new_p = np.array([[],[]])
    
    print('p:', p[:,0])
    
    for i in range(np.size(q,1)):
        if(np.abs(q[0][i])<= dist_max and np.abs(q[1][i]) <= dist_max):
            new_q = np.append(new_q, np.array([[q[0,i]], [q[1,i]]]), 1)
        if(np.abs(p[0][i]) <= dist_max and np.abs(p[1][i]) <= dist_max):
            new_p = np.append(new_p, np.array([[p[0,i]], [p[1,i]]]), 1)
            
    q = new_q
    p = new_p
    
    # Allocate vector for RMS of errors in every iteration.
    t = np.zeros([n_iter+1,1]); 
    # Start timer
    t1 = time.time()
    # Number of points
    Np = np.size(p,1);
    # Transformed data point cloud
    pt = p;
    # Vector for RMS errors in every iteration.
    ER = np.zeros([n_iter+1, 1]) 
    # Initialize temporary transform vector and matrix.
    T = np.zeros([2,1])
    R = np.eye(2)
    
    # Initialize total transform vector(s) and rotation matric(es).
    TT = np.zeros([n_iter+1,2,1])
    TR = np.ones([n_iter+1,2,2])*np.eye(2)
    #TR = np.matlib.repmat(np.eye(2), 1,1, n_iter+1)
    
    t[0] = time.time() - t1
    
    # Go into main iteration loop
    for k in range(n_iter):
        
        # Do matching
        if(Matching == 'bruteForce'):
            match, mindist = match_bruteForce(q,pt)
        elif(Matching == 'Delaunay'):
            match, mindist = match_Delaunay(q,pt,DT)
        elif(Matching == 'kDtree'):
            match, mindist = match_kDtree(q,pt)
        
        p_idx = np.ones(Np, dtype=int)
        q_idx = match
        
        if k == 0:
            ER[k] = np.sqrt( sum(pow(mindist,2))/len(mindist) )
            
        # Minimize for point
        # Determine weight vector
        weights = np.ones(len(match))
        
        R,T = eq_point( q[:,q_idx], pt[:,p_idx], weights[p_idx] )
        
        # Add to the total transformation
        TR[k+1,:,:] = np.dot(R, TR[k,:,:])
        
        TT[k+1,:,:] = np.dot(R, TT[k,:,:]) + T
        
        # Apply last transformation
        #pt = np.dot(TR[k+1,:,:], p) + np.matlib.repmat(TT[k+1,:,:], 1, Np)        
        pt = np.dot(TR[k+1,:,:], p) + TT[k+1,:,:]
        
        # Root mean of objective function 
        ER[k+1] = rms_error( q[:,q_idx], pt[:,p_idx] )
        
        t[k+1] = time.time() - t1
        
        # check error
        if np.abs(ER[k+1] - ER[k]) < tolerance:
            break
        
    return TR, TT, ER, t, pt
        

# Match Brute Force algorithm
def match_bruteForce(q, p):
    m = np.size(p,1)
    n = np.size(q,1)    
    match = np.zeros(m, dtype=int)
    mindist = np.zeros(m)
    
    for ki in range(m):
        d = np.zeros(n)
        
        for ti in range(2):
            d = d + pow( q[ti,:] - p[ti,ki], 2 )

        mindist[ki] = np.min(d)
        match[ki] = np.argmin(d)
    
    mindist = np.sqrt(mindist)
    
    return match, mindist

# Match using kDtree
def match_kDtree(dst, src):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''
    neigh = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    neigh.fit(dst.T)
    distances, indices = neigh.kneighbors(src.T, return_distance=True)
    
    return indices.ravel(), distances.ravel()

# Determine the RMS error between two point equally sized point clouds with
# point correspondance.
# ER = rms_error(p1,p2) where p1 and p2 are 3xn matrices.
def rms_error(p1,p2):
    dsq = sum( pow(p1 - p2, 2), 1 )
    return np.sqrt( np.mean(dsq) )

def eq_point(q, p, weights):
    m = np.size(p,1)
    n = np.size(q,1)
    
    # normalize weights
    weights = weights / sum(weights);
    
    # Transform 1D array into 2D array
    weights = weights[np.newaxis]

    # find data centroid and deviations from centroid
    q_bar = np.dot(q, np.transpose(weights))
    q_mark = q - q_bar
    #q_mark = q - q_bar*np.ones([2,n])
    
    # Apply weights
    #q_mark = q_mark * np.matlib.repmat(weights, 2, 1)
    
    #find data centroid and deviations from centroid
    p_bar = np.dot(p, np.transpose(weights))
    p_mark = p - p_bar
    #p_mark = p - p_bar*np.ones([2,m])
    #p_mark = p - np.matlib.repmat(p_bar, 1, m);
    
    # Apply weights
    #p_mark = p_mark * np.matlib.repmat(weights, 2, 1);

    # Sum of all pi*qi.T    
    N = np.dot(p_mark, np.transpose(q_mark))   # taking points of q in matched order
    
    U,S,Vt = np.linalg.svd(N)                  # singular value decomposition
    
    #detUVt = np.linalg.det( np.dot(U, Vt) )
    
    #R = np.dot(np.transpose(Vt), np.dot(np.diag([1, detUVt]), np.transpose(U)))
    R = np.dot( np.transpose(Vt), np.transpose(U) )
    
    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[1,:] *= -1
       R = np.dot(Vt.T, U.T)
    
    T = q_bar - np.dot(R, p_bar)
    
    return R, T


'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [match mindist] = match_Delaunay(q, p, DT)
	match = transpose(nearestNeighbor(DT, transpose(p)));
	mindist = sqrt(sum((p-q(:,match)).^2,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R,T] = eq_plane(q,p,n,weights)
n = n .* repmat(weights,3,1);
c = cross(p,n);
cn = vertcat(c,n);
C = cn*transpose(cn);
b = - [sum(sum((p-q).*repmat(cn(1,:),3,1).*n));
       sum(sum((p-q).*repmat(cn(2,:),3,1).*n));
       sum(sum((p-q).*repmat(cn(3,:),3,1).*n));
       sum(sum((p-q).*repmat(cn(4,:),3,1).*n));
       sum(sum((p-q).*repmat(cn(5,:),3,1).*n));
       sum(sum((p-q).*repmat(cn(6,:),3,1).*n))];
   
X = C\b;
cx = cos(X(1)); cy = cos(X(2)); cz = cos(X(3)); 
sx = sin(X(1)); sy = sin(X(2)); sz = sin(X(3)); 
R = [cy*cz cz*sx*sy-cx*sz cx*cz*sy+sx*sz;
     cy*sz cx*cz+sx*sy*sz cx*sy*sz-cz*sx;
     -sy cy*sx cx*cy];
    
T = X(4:6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R,T] = eq_lmaPoint(q,p)
Rx = @(a)[1     0       0;
          0     cos(a)  -sin(a);
          0     sin(a)  cos(a)];
      
Ry = @(b)[cos(b)    0   sin(b);
          0         1   0;
          -sin(b)   0   cos(b)];
      
Rz = @(g)[cos(g)    -sin(g) 0;
          sin(g)    cos(g)  0;
          0         0       1];
Rot = @(x)Rx(x(1))*Ry(x(2))*Rz(x(3));
myfun = @(x,xdata)Rot(x(1:3))*xdata+repmat(x(4:6),1,length(xdata));
options = optimset('Algorithm', 'levenberg-marquardt');
x = lsqcurvefit(myfun, zeros(6,1), p, q, [], [], options);
R = Rot(x(1:3));
T = x(4:6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extrapolation in quaternion space. Details are found in:
%
% Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes. 
% IEEE Transactions on pattern analysis and machine intelligence, 239?256.
function [dv] = extrapolate(v,d,vmax)
p1 = polyfit(v,d,1); % linear fit
p2 = polyfit(v,d,2); % parabolic fit
v1 = -p1(2)/p1(1); % linear zero crossing
v2 = -p2(2)/(2*p2(1)); % polynomial top point
if issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1])
    disp('Parabolic update!');
    dv = v2;
elseif issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
        || (v2 < 0 && issorted([0 v1 vmax]))
    disp('Line based update!');
    dv = v1;
elseif v1 > vmax && v2 > vmax
    disp('Maximum update!');
    dv = vmax;
else
    disp('No extrapolation!');
    dv = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Converts (orthogonal) rotation matrices R to (unit) quaternion
% representations
% 
% Input: A 3x3xn matrix of rotation matrices
% Output: A 4xn matrix of n corresponding quaternions
%
% http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
function quaternion = rmat2quat(R)
Qxx = R(1,1,:);
Qxy = R(1,2,:);
Qxz = R(1,3,:);
Qyx = R(2,1,:);
Qyy = R(2,2,:);
Qyz = R(2,3,:);
Qzx = R(3,1,:);
Qzy = R(3,2,:);
Qzz = R(3,3,:);
w = 0.5 * sqrt(1+Qxx+Qyy+Qzz);
x = 0.5 * sign(Qzy-Qyz) .* sqrt(1+Qxx-Qyy-Qzz);
y = 0.5 * sign(Qxz-Qzx) .* sqrt(1-Qxx+Qyy-Qzz);
z = 0.5 * sign(Qyx-Qxy) .* sqrt(1-Qxx-Qyy+Qzz);
quaternion = reshape([w;x;y;z],4,[]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Converts (unit) quaternion representations to (orthogonal) rotation matrices R
% 
% Input: A 4xn matrix of n quaternions
% Output: A 3x3xn matrix of corresponding rotation matrices
%
% http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#From_a_quaternion_to_an_orthogonal_matrix
function R = quat2rmat(quaternion)
q0(1,1,:) = quaternion(1,:);
qx(1,1,:) = quaternion(2,:);
qy(1,1,:) = quaternion(3,:);
qz(1,1,:) = quaternion(4,:);
R = [q0.^2+qx.^2-qy.^2-qz.^2 2*qx.*qy-2*q0.*qz 2*qx.*qz+2*q0.*qy;
     2*qx.*qy+2*q0.*qz q0.^2-qx.^2+qy.^2-qz.^2 2*qy.*qz-2*q0.*qx;
     2*qx.*qz-2*q0.*qy 2*qy.*qz+2*q0.*qx q0.^2-qx.^2-qy.^2+qz.^2];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Least squares normal estimation from point clouds using PCA
%
% H. Hoppe, T. DeRose, T. Duchamp, J. McDonald, and W. Stuetzle. 
% Surface reconstruction from unorganized points. 
% In Proceedings of ACM Siggraph, pages 71:78, 1992.
%
% p should be a matrix containing the horizontally concatenated column
% vectors with points. k is a scalar indicating how many neighbors the
% normal estimation is based upon.
%
% Note that for large point sets, the function performs significantly
% faster if Statistics Toolbox >= v. 7.3 is installed.
%
% Jakob Wilm 2010
function n = lsqnormest(p, k)
m = size(p,2);
n = zeros(3,m);
v = ver('stats');
if str2double(v.Version) >= 7.5 
    neighbors = transpose(knnsearch(transpose(p), transpose(p), 'k', k+1));
else
    neighbors = k_nearest_neighbors(p, p, k+1);
end
for i = 1:m
    x = p(:,neighbors(2:end, i));
    p_bar = 1/k * sum(x,2);
    
    P = (x - repmat(p_bar,1,k)) * transpose(x - repmat(p_bar,1,k)); %spd matrix P
    %P = 2*cov(x);
    
    [V,D] = eig(P);
    
    [~, idx] = min(diag(D)); % choses the smallest eigenvalue
    
    n(:,i) = V(:,idx);   % returns the corresponding eigenvector    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Program to find the k - nearest neighbors (kNN) within a set of points. 
% Distance metric used: Euclidean distance
%
% Note that this function makes repetitive use of min(), which seems to be
% more efficient than sort() for k < 30.
function [neighborIds neighborDistances] = k_nearest_neighbors(dataMatrix, queryMatrix, k)
numDataPoints = size(dataMatrix,2);
numQueryPoints = size(queryMatrix,2);
neighborIds = zeros(k,numQueryPoints);
neighborDistances = zeros(k,numQueryPoints);
D = size(dataMatrix, 1); %dimensionality of points
for i=1:numQueryPoints
    d=zeros(1,numDataPoints);
    for t=1:D % this is to avoid slow repmat()
        d=d+(dataMatrix(t,:)-queryMatrix(t,i)).^2;
    end
    for j=1:k
        [s,t] = min(d);
        neighborIds(j,i)=t;
        neighborDistances(j,i)=sqrt(s);
        d(t) = NaN; % remove found number from d
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Boundary point determination. Given a set of 3D points and a
% corresponding triangle representation, returns those point indices that
% define the border/edge of the surface.
function bound = find_bound(pts, poly)
%Correcting polygon indices and converting datatype 
poly = double(poly);
pts = double(pts);
%Calculating freeboundary points:
TR = TriRep(poly, pts(1,:)', pts(2,:)', pts(3,:)');
FF = freeBoundary(TR);
%Output
bound = FF(:,1);
'''