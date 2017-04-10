function [ dist ] = getTypicalDistance( distsAtAng,bufferSize )
%getTypicalDistance Provides the moving "average" distance of the past bufferSize
%values. This "average" calculation can be customized.

dist=movmean(distsAtAng,bufferSize);

end

