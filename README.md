<h1>Inverse Kinematics</h1>
- Scene: 2d line-segment skeleton of figure
- 2d skeleton creation</br>
  ◦ User can create new bone by selecting parent bone (by clicking on the parent's endpoint) and clicking anywhere to define
  end of bone</br>
  ◦ Bones can have more child bones</br>
  ◦ Each bone has its position, rotation angle and length (end point can be calculated)</br>
  ◦ User can select starting and ending bone of the IK sequence</br>
- Forward kinematics
  ◦ Select bone by clicking, change rotation angle by dragging</br>
  ◦ Child bones must be transformed correctly </br>
- Inverse kinematics (relaxation by gradient calculation)</br>
  ◦ User can move end bone</br>
  ◦ IK solves bones in the IK sequence
