%% Returns the derivatives of the DMP states
%  @param[in] dmp: The DMP object.
%  @param[in] x: phase variable.
%  @param[in] y: 'y' state of the DMP.
%  @param[in] z: 'z' state of the DMP.
%  @param[in] y0: initial position.
%  @param[in] g: Goal position.
%  @param[in] y_c: coupling term for the dynamical equation of the 'y' state.
%  @param[in] z_c: coupling term for the dynamical equation of the 'z' state.
%  @param[out] dy: derivative of the 'y' state of the DMP.
%  @param[out] dz: derivative of the 'z' state of the DMP.
function [dy, dz] = DMP_get_states_dot(dmp, x, y, z, y0, g, y_c, z_c)

    if (nargin < 8), y_c=0; end
    if (nargin < 7), z_c=0; end

    v_scale = dmp.get_v_scale();

    shape_attr = dmp.shape_attractor(x, y0, g);
    goal_attr = dmp.goal_attractor(x, y, z, g);

    dz = ( goal_attr + shape_attr + z_c) / v_scale;

    dy = ( z + y_c) / v_scale;

end