function [idex,inopen]=inopen(node,OPEN)
idex=0;
inopen=0;
  if  isempty(OPEN)
      inopen=0;
  else
    for i=1:size(OPEN,1)
        if((node(1,1)==OPEN(i,2))&&(node(1,2)==OPEN(i,3)))
          idex=i;
          inopen=1;
        end
    end
  end
end