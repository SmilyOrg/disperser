
width = 64
dims = 3
frame = 0;

num = width*width
coords = num*dims

%hold all

reload = true;
livereload = false;

do

  if reload || livereload
    file = fopen("build/points.bin", "r");
    nums = fread(file, Inf, "float64");
    fclose(file);
    reload = false;
    printf("reloaded\n");
  end

  frames = size(nums)(1) / coords;
  frame = max(0, min(frames-1, frame))

  offset = 1 + frame*coords;
  
  points = reshape(nums(offset:offset+coords-1), num, dims);

  scatter3(
    points(:, 1),
    points(:, 2),
    points(:, 3)
  );

  xlabel("x");
  ylabel("y");
  zlabel("z");
  axis("square");
  %axis([-1, 1, -1, 1, -1, 1], "square");
  axis([-50, 50, -50, 50, -50, 50], "square");
  axis equal

  
  %c = kbhit()
  
  %b = waitforbuttonpress()
  
  fflush(stdout);

  if livereload
    sleep(0.5);
  else
    [x, y, buttons] = ginput(1);
    
  end
   
  if buttons == 97
    frame--;
  end
  
  if buttons == 100
    frame++;
  end
  
  if buttons == 114
    reload = true;
  end 
  
until (buttons == 113)

%r = rows(points);

%points = [points(1 : 3 : r), points(2 : 3 : r), points(3 : 3 : r)];
%scatter3(points(:, 1), points(:, 2), points(:, 3));