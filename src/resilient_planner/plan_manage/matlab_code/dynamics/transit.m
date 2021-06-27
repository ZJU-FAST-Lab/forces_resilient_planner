function next = transit(z, p)
   global model
   
   x = z((model.nin + 1) : model.nvar);
   u = z(1:4);

   xnext = RK2( x, u, @nonlinear_dynamics, model.dt, p);
   next = [xnext; u];
end