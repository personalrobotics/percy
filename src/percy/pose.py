PACKAGE = 'percy'
import roslib; roslib.load_manifest(PACKAGE)
import numpy
import random
from tf.transformations import rotation_from_matrix

class PoseError(Exception):
    pass

def mean_and_variance(poses):
    if not len(poses):
        raise PoseError("No poses")
   
    M = sum(poses)/(1.0*len(poses))
    rotation = M[0:3,0:3]
    U,r_variance,V = numpy.linalg.svd(rotation)
    U = numpy.matrix(U)
    V = numpy.matrix(V)
    R = U*V
    M[0:3,0:3] = R
    
    M_t = numpy.array(M)[0:3,3]
    t_variance = sum([(M_t - numpy.array(pose)[0:3,3])**2
                       for pose in poses])/len(poses)
    
    return M, t_variance, r_variance

def pose_similarity(pose1,pose2):
    
    pose_transform = numpy.linalg.inv(pose1)*pose2
    position_distance = numpy.linalg.norm(numpy.array(pose_transform)[0:3,3])

    angle,direction,point = rotation_from_matrix(pose_transform)

    return abs(angle), position_distance


def ransac_pose_estimate(poses,
                         n_iters=0,
                         angle_inlier=0.1,
                         distance_inlier=0.02,
                         inlier_names=None):

    if inlier_names is None:
        inlier_names = range(len(poses))
    
    most_inliers = []
    most_inlier_indices = []
    n_poses = len(poses)
    
    if n_iters:
        samples = random.sample(poses,n_iters)
    else:
        samples = poses
    
    for rand_pose in samples:
        
        inliers = []
        inlier_indices = []
        
        for j in range(n_poses):
            angle, distance = pose_similarity(poses[j],rand_pose)
            if(angle < angle_inlier and
               distance < distance_inlier):
                inliers.append(poses[j])
                inlier_indices.append(inlier_names[j])

        if(len(inliers)>len(most_inliers)):
            most_inliers = inliers
            most_inlier_indices = inlier_indices
    
    M,tv,rv = mean_and_variance(most_inliers)

    return {'mean' : M,
            'inliers' : most_inlier_indices,
            'translate_variance' : tv,
            'rotation_scaling' : rv}

def offset_between_frames(poses_a,
                          poses_b,
                          angle_inlier = 0.15,
                          distance_inlier = 0.05):
    
    common_tags = list(set(poses_a.keys()) & set(poses_b.keys()))
    tag_offsets = []
    for tag in common_tags:
        print tag
        tag_offsets.append(
                #poses_b[tag]['mean'] * numpy.linalg.inv(poses_a[tag]['mean']))
                poses_b[tag]['mean'] * numpy.linalg.inv(poses_a[tag]['mean']))
    
    ransac_result = ransac_pose_estimate(
            tag_offsets,
            angle_inlier = angle_inlier,
            distance_inlier = distance_inlier,
            inlier_names = common_tags)
    
    return ransac_result

def mean(poses):
    mean, t_variance, r_variance = mean_and_variance(poses)
    return mean
