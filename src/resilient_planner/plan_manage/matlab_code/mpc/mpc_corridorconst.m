function ineq = mpc_corridorconst(z, p)

    global index model                   % global index information
    
    ego_pos  =   z(index.z.pos);         % current stage position [x, y, z]    3*1 
    
    A = reshape(p(index.p.polyConstA),[3,model.nh])';
    b = p(index.p.polyConstb);    

    ineq =  A*ego_pos-b;
end