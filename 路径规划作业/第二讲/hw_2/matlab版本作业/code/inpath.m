function isinpath=inpath(node,path)
  isinpath=0;
  for i=1:size(path,1)
      if((node(1,1)==path(i,2))&&(node(1,2)==path(i,3)))
          isinpath=1;
          break;
      end
  end
end