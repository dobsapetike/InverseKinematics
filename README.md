<h1>Inverse Kinematics</h1>
- Scene: 2d line-segment skeleton of figure
- 2d skeleton creation
  ◦ User can create new bone by selecting parent bone (by clicking on the parent's endpoint) and clicking anywhere to define
  end of bone
  ◦ Bones can have more child bones
  ◦ Each bone has its position, rotation angle and length (end point can be calculated)
  ◦ User can select starting and ending bone of the IK sequence
- Forward kinematics
  ◦ Select bone by clicking, change rotation angle by dragging
  ◦ Child bones must be transformed correctly 
- Inverse kinematics (relaxation by gradient calculation)
  ◦ User can move end bone - IK solves bones in the IK sequence
