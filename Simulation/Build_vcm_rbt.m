function vcm = Build_vcm_rbt(i)

vcm = rigidBodyTree("DataFormat","row");

vcm_body = rigidBody(sprintf("vcm_body_%d",i));
vcm_joint = rigidBodyJoint(sprintf("vcm_joint_%d",i),"fixed");

%vcm_joint.JointAxis = [0,0,1];

vcm_body.Joint = vcm_joint;

setFixedTransform(vcm_joint,trvec2tform([0,0, 0.06395/2]));

% Add the collision bodies to the rigid body objects
collAct = collisionCylinder(0.0224, 0.06395);
addCollision(vcm_body,collAct);
addVisual(vcm_body, "cylinder", [0.0224, 0.06395]);

addBody(vcm,vcm_body,"base");

end