#include <H5Cpp.h>
#include <sys/stat.h>
#include <vector>
#include <iostream>

using namespace H5;

const std::string HDF_PATH = "data.h5";
const int N = 40;

// HDF5 handles
H5File* file = nullptr;
DataSet* ds_xcurr = nullptr;
DataSet* ds_xref = nullptr;
DataSet* ds_uopt = nullptr;
DataSet* ds_xnext = nullptr;
hsize_t current_size[2] = {0, 12};
hsize_t ref_dims[2] = {0, 12*(N+1)};

const hsize_t STATE_CHUNK[2] = {100, 12};
const hsize_t REF_CHUNK[2] = {100, 12*(N+1)};
const hsize_t U_CHUNK[2] = {100, 8};

// Data buffers
std::vector<double> x_current_buf, x_ref_buf, u_opt_buf, x_next_buf;

// HDF5 Management
void write_chunk() {
    if (!ds_xcurr || !ds_xref || !ds_uopt || !ds_xnext) {
        std::cerr << "Dataset pointers are invalid!" << std::endl;
        return;
    }
    if (x_current_buf.empty()) return;

    const hsize_t n_new = x_current_buf.size() / 12;
    
    // Extend datasets
    current_size[0] += n_new;
    ref_dims[0] += n_new;
    
    ds_xcurr->extend(current_size);
    ds_xref->extend(ref_dims);
    
    hsize_t u_new_size[2] = {current_size[0], 8};
    ds_uopt->extend(u_new_size);
    
    ds_xnext->extend(current_size);

    // Write x_current
    hsize_t offset[2] = {current_size[0] - n_new, 0};
    hsize_t count[2] = {n_new, 12};
    DataSpace mem_space(2, count);
    
    // Get updated file space after extension
    DataSpace file_space_xcurr = ds_xcurr->getSpace();
    file_space_xcurr.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xcurr->write(x_current_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space_xcurr);

    // Write x_ref
    hsize_t ref_count[2] = {n_new, 12*(N+1)};
    DataSpace ref_mem_space(2, ref_count);
    DataSpace file_space_xref = ds_xref->getSpace();
    file_space_xref.selectHyperslab(H5S_SELECT_SET, ref_count, offset);
    ds_xref->write(x_ref_buf.data(), PredType::NATIVE_DOUBLE, ref_mem_space, file_space_xref);

    // Write u_opt
    hsize_t u_count[2] = {n_new, 8};
    DataSpace u_mem_space(2, u_count);
    DataSpace file_space_uopt = ds_uopt->getSpace();
    file_space_uopt.selectHyperslab(H5S_SELECT_SET, u_count, offset);
    ds_uopt->write(u_opt_buf.data(), PredType::NATIVE_DOUBLE, u_mem_space, file_space_uopt);

    // Write x_next
    DataSpace file_space_xnext = ds_xnext->getSpace();
    file_space_xnext.selectHyperslab(H5S_SELECT_SET, count, offset);
    ds_xnext->write(x_next_buf.data(), PredType::NATIVE_DOUBLE, mem_space, file_space_xnext);

    // Clear buffers
    x_current_buf.clear();
    x_ref_buf.clear();
    u_opt_buf.clear();
    x_next_buf.clear();
    std::cout << "\nChunk is written and cleared...\n";
}

void cleanup_hdf5() {
    try {
        // Write any remaining data first
        if (!x_current_buf.empty()) {
            write_chunk();
        }
        
        // Explicitly flush before closing
        if (file) {
            H5Fflush(file->getId(), H5F_SCOPE_GLOBAL);
        }

        // Then delete resources
        if (ds_xnext) { delete ds_xnext; ds_xnext = nullptr; }
        if (ds_uopt)  { delete ds_uopt;  ds_uopt  = nullptr; }
        if (ds_xref)  { delete ds_xref;  ds_xref  = nullptr; }
        if (ds_xcurr) { delete ds_xcurr; ds_xcurr = nullptr; }
        if (file)     { delete file;     file     = nullptr; }
        
    } catch (...) {
        std::cerr << "Error during final cleanup\n";
    }
}

void initialize_hdf5() {
    try {
        struct stat buffer;
        bool file_exists = (stat(HDF_PATH.c_str(), &buffer) == 0);

        if(file_exists) {
            file = new H5File(HDF_PATH, H5F_ACC_RDWR);
            
            // Open existing datasets
            ds_xcurr = new DataSet(file->openDataSet("x_current"));
            ds_xref = new DataSet(file->openDataSet("x_ref"));
            ds_uopt = new DataSet(file->openDataSet("u_opt"));
            ds_xnext = new DataSet(file->openDataSet("x_next"));
            
        // Get current dimensions for x_ref
        {
            DataSpace space = ds_xref->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            ref_dims[0] = dims[0];
            ref_dims[1] = dims[1];  // should be 12*(N+1)
            std::cout << "x_ref size: " << ref_dims[0] << " x " << ref_dims[1] << "\n";
        }
        // For u_opt, track its first dimension separately, e.g., u_dims
        {
            DataSpace space = ds_uopt->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            // You may want a separate array, e.g. u_dims
            // But if you use current_size[0] for number of rows and second dim is fixed 8,
            // ensure they match or handle accordingly.
            std::cout << "u_opt size: " << dims[0] << " x " << dims[1] << "\n";
        }
        // x_next has same shape as x_current
        {
            DataSpace space = ds_xnext->getSpace();
            hsize_t dims[2];
            space.getSimpleExtentDims(dims);
            std::cout << "x_next size: " << dims[0] << " x " << dims[1] << "\n";
        }
        } else {
            file = new H5File(HDF_PATH, H5F_ACC_TRUNC);
            
            // Create dataspace for extendible datasets
            hsize_t init_dims[2] = {0, 12};
            hsize_t max_dims[2] = {H5S_UNLIMITED, 12};
            DataSpace x_space(2, init_dims, max_dims);
            
            hsize_t ref_init[2] = {0, 12*(N+1)};
            hsize_t ref_max[2] = {H5S_UNLIMITED, 12*(N+1)};
            DataSpace ref_space(2, ref_init, ref_max);
            
            hsize_t u_init[2] = {0, 8};
            hsize_t u_max[2] = {H5S_UNLIMITED, 8};
            DataSpace u_space(2, u_init, u_max);

            // Add chunking properties
            DSetCreatPropList props;
            props.setChunk(2, STATE_CHUNK);
            
            DSetCreatPropList ref_props;
            ref_props.setChunk(2, REF_CHUNK);
            
            DSetCreatPropList u_props;
            u_props.setChunk(2, U_CHUNK);

            // Create datasets with chunking
            ds_xcurr = new DataSet(file->createDataSet("x_current", 
                PredType::NATIVE_DOUBLE, x_space, props));
            ds_xref = new DataSet(file->createDataSet("x_ref", 
                PredType::NATIVE_DOUBLE, ref_space, ref_props));
            ds_uopt = new DataSet(file->createDataSet("u_opt", 
                PredType::NATIVE_DOUBLE, u_space, u_props));
            ds_xnext = new DataSet(file->createDataSet("x_next", 
                PredType::NATIVE_DOUBLE, x_space, props));
        }
    } catch (const Exception& e) {
        std::cerr << "HDF5 Error: " << e.getCDetailMsg() << "\n";
        exit(1);
    }
}
