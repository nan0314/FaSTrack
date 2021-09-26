from Grid.GridProcessing import Grid
import numpy as np

"""
NOTE: THIS IS NOT MY ORIGINAL CODE. It was adopted from the Matlab .m
files from helperOC Optimal Control toolbox. I simply rewrote it in python
and added some documentation. See the original .m file at
https://github.com/HJReachability/helperOC.git in helperOC/valFuncs/proj.m
"""

def proj(g, data, dimsToRemove, xs='min', NOut = -1, process=True):
# [gOut, dataOut] = proj(g, data, dims, xs, NOut)
#   Projects data corresponding to the grid g in g.dim dimensions, removing
#   dimensions specified in dims. If a point is specified, a slice of the
#   full-dimensional data at the point xs is taken.
#
# Inputs:
#   g            - grid corresponding to input data
#   data         - input data
#   dimsToRemove - vector of length g.dim specifying dimensions to project
#                 For example, if g.dim = 4, then dims = [0 0 1 1] would
#                 project the last two dimensions
#   xs           - Type of projection (defaults to 'min')
#       'min':    takes the union across the projected dimensions
#       'max':    takes the intersection across the projected dimensions
#       a vector: takes a slice of the data at the point xs
#   NOut    - number of grid points in output grid (defaults to the same
#             number of grid points of the original grid in the unprojected
#             dimensions)
#   process            - specifies whether to call processGrid to generate
#                        grid points
#
# Outputs:
#   gOut    - grid corresponding to projected data
#   dataOut - projected data
#
# See proj_test.m

    ## Input checking
    if len(dimsToRemove) != g.dims:
        raise RuntimeError('Dimensions are inconsistentnot ')
    

    if sum(dimsToRemove) == 0:
        raise RuntimeWarning('Input and output dimensions are the samenot ')
        return [g, data]
    

    # If a slice is requested, make sure the specified point has the correct
    # dimension
    if not xs.isalpha() and len(xs) != sum(dimsToRemove):
        raise RuntimeError('Dimension of xs and dims do not matchnot ')
    
    if (NOut == -1):
        NOut = []
        for i in range(len(dimsToRemove)):
            if not dimsToRemove[i]:
                NOut.append(g.pts_each_dim[i])

    

    dataDims = len(data.shape)
    # if np.count_nonzero(data) and not (dataDims == g.dims or dataDims == g.dims+1) and not iscell(data):
    #     raise RuntimeError('Inconsistent input data dimensionsnot ')
    

    ## Project data
    gOut = g
    dataOut = data
    if dataDims == g.dims:
        [gOut, dataOut] = projSingle(g, data, dimsToRemove, xs, NOut, process)
    
    ## TODO: Finish implementing proj function. Following code is still matlab code ##

    # else: # dataDims == g.dim + 1
    #     # Project grid
    #     gOut = projSingle(g, [], dimsToRemove, xs, NOut, process,1)
    
    #     # Project data
    #     if iscell(data)
    #         numTimeSteps = length(data)
    #     else
    #         numTimeSteps = size(data, dataDims)
    #         colonsIn = repmat({':'}, 1, g.dim)
        
        
    #     dataOut = zeros([NOut' numTimeSteps])
    #     colonsOut = repmat({':'}, 1, gOut.dim)
        
    #     for i = 1:numTimeSteps
    #         if iscell(data)
    #         [not , dataOut(colonsOut{:},i)] = ...
    #         projSingle(g, data{i}, dimsToRemove, xs, NOut, process)   
    #         else
    #         [not , dataOut(colonsOut{:},i)] = ...
    #         projSingle(g, data(colonsIn{:},i), dimsToRemove, xs, NOut, process)      
    return gOut, dataOut
        
    
    

def projSingle(g, data, dims, xs, NOut, process,nargout=2):
# [gOut, dataOut] = proj(g, data, dims, xs, NOut)
#   Projects data corresponding to the grid g in g.dim dimensions, removing
#   dimensions specified in dims. If a point is specified, a slice of the
#   full-dimensional data at the point xs is taken.
#
# Inputs:
#   g       - grid corresponding to input data
#   data    - input data
#   dims    - vector of length g.dim specifying dimensions to project
#                 For example, if g.dim = 4, then dims = [0 0 1 1] would
#                 project the last two dimensions
#   xs      - Type of projection (defaults to 'min')
#       'min':    takes the union across the projected dimensions
#       'max':    takes the intersection across the projected dimensions
#       a vector: takes a slice of the data at the point xs
#   NOut    - number of grid points in output grid (defaults to the same
#             number of grid points of the original grid in the unprojected
#             dimensions)
#   process            - specifies whether to call processGrid to generate
#                        grid points
#
# Outputs:
#   gOut    - grid corresponding to projected data
#   dataOut - projected data
#
# Note: Projection slicing functionality still not re-implemented into Python.
#       Original matlab code still in place.

    ## Create ouptut grid by keeping dimensions that we are not collapsing
    gOut = g
    if not np.count_nonzero(g):
        if not xs.isalpha() or (not (xs=='max') and not (xs=='min')):
            raise RuntimeError('Must perform min or max projection when not specifying grid!')
        
    else:
        proj_dims = 0
        proj_min = []
        proj_max = []
        proj_NOut = NOut

        for i in range(len(dims)):
            if not dims[i]:
               proj_dims+=1
               proj_min.append(g.min[i])
               proj_max.append(g.max[i])
    
        if len(NOut) == 1:
            proj_NOut = NOut*np.ones((proj_dims,1))
        else:
            proj_NOut = NOut
        
        
        # Process the grid to populate the remaining fields if necessary
        gOut = Grid(np.array(proj_min),np.array(proj_max),proj_dims,np.array(proj_NOut))
        
        
        # Only compute the grid if value function is not requested
        if nargout < 2:
            return gOut
    
    

    ## 'min' or 'max'
    if xs.isalpha():
        dimsToProj = np.where(dims)

        for i in range( len(dimsToProj) - 1, -1, -1):
            if xs=='min':
                data = np.squeeze(np.amin(data, dimsToProj[i][0]))
                
            elif xs=='max':
                data = np.squeeze(np.amax(data, dimsToProj[i][0]))
            else:
                raise RuntimeError('xs must be a vector, ''min'', or ''max''!')
            
        
        
        dataOut = data
        return gOut, dataOut
    
    ## TODO: Finish implementing proj function. Following code is still matlab code ##

    ## Take a slice
    # Preprocess periodic dimensions
    # [g, data] = augmentPeriodicData(g, data)

    # # temp = interpn(g.vs{1}, g.vs{2}, g.vs{3}, g.vs{4}, data, g.vs{1}, xs(1), ...
    # #   g.vs{3}, xs(2))
    # eval_pt = cell(g.dim, 1)
    # xsi = 1
    # for i in range(g.dims):
    #     if dims[i]:
    #         # If this dimension is periodic, wrap the input point to the correct period
    #         if isfield(g, 'bdry') && isequal(g.bdry{i}, @addGhostPeriodic)
    #         period = max(g.vs{i}) - min(g.vs{i})
            
    #         while xs(xsi) > max(g.vs{i})
    #             xs(xsi) = xs(xsi) - period
            
            
    #         while xs(xsi) < min(g.vs{i})
    #             xs(xsi) = xs(xsi) + period
            
            
            
    #         eval_pt{i} = xs(xsi)
    #         xsi = xsi + 1
            
    #     else
    #         eval_pt{i} = g.vs{i}
    
    
    
    # temp = interpn(g.vs{:}, data, eval_pt{:})

    # dataOut = squeeze(temp)

    # # interpn(g.vs{1}, g.vs{3}, dataOut, gOut.xs{1}, gOut.xs{2})
    # dataOut = interpn(g.vs{~dims}, dataOut, gOut.xs{:})

    return gOut, dataOut
    