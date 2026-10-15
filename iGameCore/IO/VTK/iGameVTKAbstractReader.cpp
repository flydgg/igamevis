#include "iGameVTKAbstractReader.h"
#include "iGameByteSwap.h"
#include <chrono>
#include <vector>
IGAME_NAMESPACE_BEGIN

namespace {
bool IsVtkCellSizeValid(VTKAbstractReader::VTKTYPE type, int size) {
    switch (type) {
        case VTKAbstractReader::VERTEX:
            return size == 1;
        case VTKAbstractReader::LINE:
            return size == 2;
        case VTKAbstractReader::POLYLINE:
            return size >= 2;
        case VTKAbstractReader::TRIANGLE:
            return size == 3;
        case VTKAbstractReader::POLYGON:
            return size >= 3;
        case VTKAbstractReader::PIXEL:
        case VTKAbstractReader::QUAD:
        case VTKAbstractReader::TETRA:
            return size == 4;
        case VTKAbstractReader::VOXEL:
        case VTKAbstractReader::HEXAHEDRON:
            return size == 8;
        case VTKAbstractReader::WEDGE:
            return size == 6;
        case VTKAbstractReader::PYRAMID:
            return size == 5;
        case VTKAbstractReader::PENTAGONAL_PRISM:
            return size == 10;
        case VTKAbstractReader::HEXAGONAL_PRISM:
            return size == 12;
        case VTKAbstractReader::QUADRATIC_EDGE:
            return size == 3;
        case VTKAbstractReader::QUADRATIC_TRIANGLE:
            return size == 6;
        case VTKAbstractReader::QUADRATIC_QUAD:
            return size == 8;
        case VTKAbstractReader::QUADRATIC_TETRA:
            return size == 10;
        case VTKAbstractReader::QUADRATIC_HEXAHEDRON:
            return size == 20;
        case VTKAbstractReader::QUADRATIC_WEDGE:
            return size == 15;
        case VTKAbstractReader::QUADRATIC_PYRAMID:
            return size == 13;
        default:
            // Polyhedra and Lagrange cells have variable connectivity sizes.
            return true;
    }
}

VTKAbstractReader::VTKTYPE NormalizeVtkCellType(VTKAbstractReader::VTKTYPE type, int size) {
    // Some third-party writers keep all midside nodes but incorrectly label
    // the cell with its linear VTK type.  ParaView tolerates these files.  The
    // node counts below identify the corresponding quadratic type without
    // weakening validation for arbitrary malformed connectivity.
    switch (type) {
        case VTKAbstractReader::LINE:
            if (size == 3) return VTKAbstractReader::QUADRATIC_EDGE;
            break;
        case VTKAbstractReader::TRIANGLE:
            if (size == 6) return VTKAbstractReader::QUADRATIC_TRIANGLE;
            break;
        case VTKAbstractReader::QUAD:
            if (size == 8) return VTKAbstractReader::QUADRATIC_QUAD;
            break;
        case VTKAbstractReader::TETRA:
            if (size == 10) return VTKAbstractReader::QUADRATIC_TETRA;
            break;
        case VTKAbstractReader::HEXAHEDRON:
            if (size == 20) return VTKAbstractReader::QUADRATIC_HEXAHEDRON;
            break;
        case VTKAbstractReader::WEDGE:
            if (size == 15) return VTKAbstractReader::QUADRATIC_WEDGE;
            break;
        case VTKAbstractReader::PYRAMID:
            if (size == 13) return VTKAbstractReader::QUADRATIC_PYRAMID;
            break;
        default:
            break;
    }
    return type;
}
} // namespace

VTKAbstractReader::VTKAbstractReader() {
    this->Header = nullptr;
    this->FileMajorVersion = 0;
    this->FileMinorVersion = 0;
    this->FileVersion = 0;
}
VTKAbstractReader::~VTKAbstractReader() {
    if (this->Header) delete[] this->Header;
}

int VTKAbstractReader::ReadHeader() {
    char line[256];
    igDebug("Reading vtk file header");
    //
    // read header
    //
    if (!this->ReadLine(line)) {
        igError("Premature EOF reading first line! ");
        return 0;
    }
    const int VERSION_PREFIX_LENGTH = 22;
    if (strncmp("# vtk DataFile Version", line, VERSION_PREFIX_LENGTH) != 0) {
        igError("Unrecognized file type! ");
        return 0;
    }
    if (sscanf(line + VERSION_PREFIX_LENGTH, "%d.%d", &this->FileMajorVersion, &this->FileMinorVersion) != 2) {
        igDebug("Cannot read file version:!");
        this->FileMajorVersion = 0;
        this->FileMinorVersion = 0;
    }
    if (this->FileMajorVersion > vtkLegacyReaderMajorVersion ||
        (this->FileMajorVersion == vtkLegacyReaderMajorVersion &&
         this->FileMinorVersion > vtkLegacyReaderMinorVersion)) {
        // newer file than the reader version
        igDebug("Reading file version: {}.{} with older reader version {}.{}", this->FileMajorVersion,
                this->FileMinorVersion, vtkLegacyReaderMajorVersion, vtkLegacyReaderMinorVersion);
    }
    // Compose FileVersion
    this->FileVersion = 10 * this->FileMajorVersion + this->FileMinorVersion;

    //
    // read title
    //
    if (!this->ReadLine(line)) {
        igError("Premature EOF reading title! ");
        return 0;
    }
    //delete[] this->Header;
    this->Header = new char[strlen(line) + 1];
    strcpy(this->Header, line);

    igDebug("Reading vtk file entitled!{}", line);
    //
    // read type
    //
    if (!this->ReadString(line)) {
        //igError("Premature EOF reading file type!"
        //	<< " for file: " << (fname ? fname : "(Null FileName)"));
        return 0;
    }

    if (!strncmp(this->LowerCase(line), "ascii", 5)) {
        m_FileType = IGAME_ASCII;
    } else if (!strncmp(line, "binary", 6)) {
        m_FileType = IGAME_BINARY;
    } else {
        //igError("Unrecognized file type: " << line
        //	<< " for file: " << (fname ? fname : "(Null FileName)"));
        m_FileType = 0;
        return 0;
    }
    return 1;
}


//------------------------------------------------------------------------------
// Read point coordinates. Return 0 if error.
int VTKAbstractReader::ReadPointCoordinates(Points::Pointer points, int PointsNum) {
    char line[256];
    FloatArray::Pointer floatData;
    DoubleArray::Pointer doubleData;
    if (!this->ReadString(line)) {
        //igError("Cannot read points type!"
        //	<< " for file: " << (fname ? fname : "(Null FileName)"));
        return 0;
    }
    if (!strcmp(this->LowerCase(line), "double")) {
        doubleData = DynamicCast<DoubleArray>(this->ReadArray("double", PointsNum, 3));
    } else {
        floatData = DynamicCast<FloatArray>(this->ReadArray("float", PointsNum, 3));
    }
    if (floatData != nullptr) {
        // 批量写入：Points 的内存布局就是交错的 [x,y,z]×n（Vector3f），
        // 源数组同布局，直接一次 memcpy，避免每点一次 AddPoint 调用。
        points->SetNumberOfPoints(PointsNum);
        const float* src = floatData->RawPointer();
        float* dst = points->RawPointer();
        if (src != nullptr && dst != nullptr) {
            std::memcpy(dst, src, sizeof(float) * 3 * static_cast<size_t>(PointsNum));
        }
    } else if (doubleData != nullptr) {
        points->SetNumberOfPoints(PointsNum);
        const double* src = doubleData->RawPointer();
        float* dst = points->RawPointer();
        if (src != nullptr && dst != nullptr) {
            for (IGsize i = 0; i < static_cast<IGsize>(PointsNum) * 3; ++i) {
                dst[i] = static_cast<float>(src[i]);
            }
        }
    } else {
        return 0;
    }
    igDebug("Read {} points", points->GetNumberOfPoints());
    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read the cell data of a vtk data file. The number of cells (from the
// dataset) must match the number of cells defined in cell attributes (unless
// no geometry was defined).
int VTKAbstractReader::ReadCellData(int CellsNum) {
    char line[256];
    this->DataType = CELL_TYPE;
    igDebug("Reading vtk cell data");
    //
    // Read keywords until end-of-file
    //
    while (this->ReadString(line)) {
        if (line[0] == '\0') { continue; }
        //
        // read scalar data
        //
        if (!strncmp(this->LowerCase(line), "scalars", 7)) {
            if (!this->ReadScalarData(CellsNum)) { return 0; }
        }
        //
        // read vector data
        //
        else if (!strncmp(line, "vectors", 7)) {
            if (!this->ReadVectorData(CellsNum)) { return 0; }
        }
        //
        // read 3x2 symmetric tensor data
        //
        else if (!strncmp(line, "tensors6", 8)) {
            if (!this->ReadTensorData(CellsNum, 6)) { return 0; }
        }
        //
        // read 3x3 tensor data
        //
        else if (!strncmp(line, "tensors", 7)) {
            if (!this->ReadTensorData(CellsNum)) { return 0; }
        }
        //
        // read normals data
        //
        else if (!strncmp(line, "normals", 7)) {
            if (!this->ReadNormalData(CellsNum)) { return 0; }
        }
        //
        // read texture coordinates data
        //
        else if (!strncmp(line, "texture_coordinates", 19)) {
            if (!this->ReadTCoordsData(CellsNum)) { return 0; }
        }
        //
        // read the global id data
        //
        else if (!strncmp(line, "global_ids", 10)) {
            if (!this->ReadGlobalIds(CellsNum)) { return 0; }
        }
        //
        // read the pedigree id data
        //
        else if (!strncmp(line, "pedigree_ids", 12)) {
            if (!this->ReadPedigreeIds(CellsNum)) { return 0; }
        }
        //
        // read color scalars data
        //
        else if (!strncmp(line, "color_scalars", 13)) {
            if (!this->ReadCoScalarData(CellsNum)) { return 0; }
        }
        //
        // read lookup table. Associate with scalar data.
        //
        else if (!strncmp(line, "lookup_table", 12)) {
            if (!this->ReadLutData()) { return 0; }
        }
        //
        // read field of data
        //
        else if (!strncmp(line, "field", 5)) {
            this->ReadFieldData();
        }
        //
        // maybe bumped into point data
        //
        else if (!strncmp(line, "point_data", 10)) {
            int npts;
            if (!this->Read(&npts)) {
                //igError("Cannot read point data!");
                return 0;
            }
            this->ReadPointData(npts);
            break;
        } else if (!strncmp(line, "metadata", 8)) {
            if (!ProcessMetaData()) { igDebug("Read metadata error!"); }
            continue;
        } else {
            //igError("Unsupported cell attribute type: " << line
            //	<< " for file: " << (fname ? fname : "(Null FileName)"));
            return 0;
        }
    }
    this->UpdateReadProgress();
    return 1;
}


//------------------------------------------------------------------------------
// Read the point data of a vtk data file. The number of points (from the
// dataset) must match the number of points defined in point attributes (unless
// no geometry was defined).
int VTKAbstractReader::ReadPointData(int PointsNum) {
    char line[256];
    AttributeSet::Pointer PointData = m_Data.GetData();

    this->DataType = POINT_TYPE;
    igDebug("Reading vtk point data");
    //
    // Read keywords until end-of-file
    //
    while (this->ReadString(line)) {
        if (line[0] == '\0') { continue; }
        //
        // read scalar data
        //
        if (!strncmp(this->LowerCase(line), "scalars", 7)) {
            if (!this->ReadScalarData(PointsNum)) { return 0; }
        }
        //
        // read vector data
        //
        else if (!strncmp(line, "vectors", 7)) {
            if (!this->ReadVectorData(PointsNum)) { return 0; }
        }
        //
        // read 3x2 symmetric tensor data
        //
        else if (!strncmp(line, "tensors6", 8)) {
            if (!this->ReadTensorData(PointsNum, 6)) { return 0; }
        }
        //
        // read 3x3 tensor data
        //
        else if (!strncmp(line, "tensors", 7)) {
            if (!this->ReadTensorData(PointsNum)) { return 0; }
        }
        //
        // read normals data
        //
        else if (!strncmp(line, "normals", 7)) {
            if (!this->ReadNormalData(PointsNum)) { return 0; }
        }
        //
        // read texture coordinates data
        //
        else if (!strncmp(line, "texture_coordinates", 19)) {
            if (!this->ReadTCoordsData(PointsNum)) { return 0; }
        }
        //
        // read the global id data
        //
        else if (!strncmp(line, "global_ids", 10)) {
            if (!this->ReadGlobalIds(PointsNum)) { return 0; }
        }
        //
        // read the pedigree id data
        //
        else if (!strncmp(line, "pedigree_ids", 12)) {
            if (!this->ReadPedigreeIds(PointsNum)) { return 0; }
        }
        //
        // read the edge flags data
        //
        else if (!strncmp(line, "edge_flags", 10)) {
            if (!this->ReadEdgeFlags(PointsNum)) { return 0; }
        }
        //
        // read color scalars data
        //
        else if (!strncmp(line, "color_scalars", 13)) {
            if (!this->ReadCoScalarData(PointsNum)) { return 0; }
        }
        //
        // read lookup table. Associate with scalar data.
        //
        else if (!strncmp(line, "lookup_table", 12)) {
            if (!this->ReadLutData()) { return 0; }
        }
        //
        // read field of data
        //
        else if (!strncmp(line, "field", 5)) {
            this->ReadFieldData();
        }
        //
        // maybe bumped into cell data
        //
        else if (!strncmp(line, "cell_data", 9)) {
            int ncells;
            if (!this->Read(&ncells)) {
                igError("Cannot read cell data!");
                return 0;
            }
            this->ReadCellData(ncells);
            break;
        } else if (!strncmp(this->LowerCase(line), "metadata", 8)) {
            if (!ProcessMetaData()) { igDebug("Read metadata error!"); }
            continue;
        } else {
            //igError("Unsupported point attribute type: " << line
            //	<< " for file: " << (fname ? fname : "(Null FileName)"));
            return 0;
        }
    }
    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read scalar point attributes. Return 0 if error.
int VTKAbstractReader::ReadScalarData(int PointsNum) {
    char line[256], name[256], key[256], tableName[256];
    int numComp = 1;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    if (!this->ReadString(key)) { return 0; }

    // the next string could be an integer number of components or a lookup table
    if (strcmp(this->LowerCase(key), "lookup_table") != 0) {
        numComp = atoi(key);
        if (numComp < 1 || !this->ReadString(key)) { return 0; }
    }

    if (strcmp(this->LowerCase(key), "lookup_table") != 0) { return 0; }

    if (!this->ReadString(tableName)) { return 0; }

    // Read the data
    auto data = this->ReadArray(line, PointsNum, numComp);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddScalar(IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddScalar(IG_CELL, data);
        }
    } else {
        return 0;
    }
    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read vector point attributes. Return 0 if error.
int VTKAbstractReader::ReadVectorData(int PointsNum) {
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);


    data = this->ReadArray(line, PointsNum, 3);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_VECTOR, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_VECTOR, IG_CELL, data);
        }
    } else {
        return 0;
    }
    this->UpdateReadProgress();

    return 1;
}

//------------------------------------------------------------------------------
// Read normal point attributes. Return 0 if error.
int VTKAbstractReader::ReadNormalData(int PointsNum) {
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    data = this->ReadArray(line, PointsNum, 3);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_NORMAL, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_NORMAL, IG_CELL, data);
        }
    } else {
        return 0;
    }

    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read tensor point attributes. Return 0 if error.
int VTKAbstractReader::ReadTensorData(int PointsNum, int numComp) {
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    data = this->ReadArray(line, PointsNum, numComp);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_TENSOR, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_TENSOR, IG_CELL, data);
        }
    } else {
        return 0;
    }
    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read color scalar point attributes. Return 0 if error.
int VTKAbstractReader::ReadCoScalarData(int PointsNum) {
    int i, j, idx, numComp = 0;
    char name[256];
    char buffer[256];

    if (!(this->ReadString(buffer) && this->Read(&numComp))) {
        //igError("Cannot read color scalar data!"
        //	<< " for file: " << (fname ? fname : "(Null FileName)"));
        return 0;
    }
    this->DecodeString(name, buffer);

    // handle binary different from ASCII since they are stored
    // in a different format float versus uchar
    if (m_FileType == IGAME_BINARY) {
        ArrayObject::Pointer data;
        char type[14] = "unsigned_char";
        data = this->ReadArray(type, PointsNum, numComp);
        if (data != nullptr) {
            data->SetName(name);
            if (this->DataType == POINT_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, data);
            } else if (this->DataType == CELL_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, data);
            }
        } else {
            return 0;
        }
    } else {
        ArrayObject::Pointer data;
        char type[6] = "float";
        data = this->ReadArray(type, PointsNum, numComp);
        if (data != nullptr) {
            UnsignedCharArray::Pointer scalars = UnsignedCharArray::New();
            scalars->SetDimension(numComp);
            scalars->Resize(PointsNum);
            scalars->SetName(name);
            for (i = 0; i < PointsNum; i++) {
                for (j = 0; j < numComp; j++) {
                    idx = i * numComp + j;
                    scalars->SetValue(idx, (unsigned char) (255.0 * data->GetValue(idx) + 0.5));
                }
            }
            if (this->DataType == POINT_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, scalars);
            } else if (this->DataType == CELL_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, scalars);
            }
        } else {
            return 0;
        }
    }

    this->UpdateReadProgress();

    return 1;
}

//------------------------------------------------------------------------------
// Read texture coordinates point attributes. Return 0 if error.
int VTKAbstractReader::ReadTCoordsData(int PointsNum) {
    int dim = 0;
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->Read(&dim) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    if (dim < 1 || dim > 3) { return 0; }


    data = this->ReadArray(line, PointsNum, dim);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_TCOORD, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_TCOORD, IG_CELL, data);
        }
    } else {
        return 0;
    }
    this->UpdateReadProgress();

    return 1;
}

//------------------------------------------------------------------------------
// Read texture coordinates point attributes. Return 0 if error.
int VTKAbstractReader::ReadGlobalIds(int PointsNum) {
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);
    data = this->ReadArray(line, PointsNum, 1);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, data);
        }
    } else {
        return 0;
    }

    this->UpdateReadProgress();

    return 1;
}

//------------------------------------------------------------------------------
// Read pedigree ids. Return 0 if error.
int VTKAbstractReader::ReadPedigreeIds(int PointsNum) {
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    data = this->ReadArray(line, PointsNum, 1);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, data);
        }
    } else {
        return 0;
    }

    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read edge flags. Return 0 if error.
int VTKAbstractReader::ReadEdgeFlags(int PointsNum) {
    int skipEdgeFlags = 0;
    char line[256], name[256];
    ArrayObject::Pointer data;
    char buffer[256];

    if (!(this->ReadString(buffer) && this->ReadString(line))) { return 0; }
    this->DecodeString(name, buffer);

    data = this->ReadArray(line, PointsNum, 1);
    if (data != nullptr) {
        data->SetName(name);
        if (this->DataType == POINT_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, data);
        } else if (this->DataType == CELL_TYPE) {
            m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, data);
        }

    } else {
        return 0;
    }

    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
// Read lookup table. Return 0 if error.
int VTKAbstractReader::ReadLutData() {
    // temporarily not used ,wil be processed in fulturt
    igDebug("temporarily not process this type");
    this->UpdateReadProgress();
    return 1;
}

//------------------------------------------------------------------------------
int VTKAbstractReader::ReadFieldData() {
    int i, numArrays = 0, skipField = 0;

    char name[256], type[256];
    int numComp, numTuples;
    ArrayObject::Pointer data;

    if (!(this->ReadString(name) && this->Read(&numArrays))) { return 0; }
    // Read the number of arrays specified
    for (i = 0; i < numArrays; i++) {
        char buffer[256];
        this->ReadString(buffer);
        if (strcmp(this->LowerCase(buffer), "null_array") == 0) { continue; }
        if (strcmp(buffer, "metadata") == 0) {
            i--;
            ProcessMetaData();
            continue;
        }
        this->DecodeString(name, buffer);
        this->Read(&numComp);
        this->Read(&numTuples);
        this->ReadString(type);
        if (!strncmp(this->LowerCase(type), "string", 6)) {
            if (this->m_FileType == IGAME_BINARY) {
                this->ReadLine(buffer);
            } else {
                for (int i = 0; i < numComp * numTuples; i++) { this->ReadString(buffer); }
            }
            continue;
        }
        data = this->ReadArray(type, numTuples, numComp);
        if (data != nullptr) {
            data->SetName(name);
            if (this->DataType == POINT_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_POINT, data);
            } else if (this->DataType == CELL_TYPE) {
                m_Data.GetData()->AddAttribute(IG_SCALAR, IG_CELL, data);
            }
        }
    }
    this->UpdateReadProgress();
    return 1;
}
bool VTKAbstractReader::ProcessMetaData() {
    char buffer[IGAME_CELL_MAX_SIZE];
    int infoNum = 0;
    if (!this->ReadString(buffer) || strncmp(this->LowerCase(buffer), "information", 6)) {
        igError("metadata error!");
        return false;
    }
    if (!this->Read(&infoNum)) {
        igError("metadata error!");
        return false;
    }
    for (int i = 0; i < infoNum; i++) {
        if (!this->ReadLine(buffer)) return false;
        if (!this->ReadLine(buffer)) return false;
    }
    return true;
}
namespace {
/** 跳过 ASCII 空白后判断下一个 token 是否等于 kw（只读，不移动读取指针） */
bool PeekTokenIs(const char* is, const char* end, const char* kw) {
    if (is == nullptr || end == nullptr) { return false; }
    while (is < end && (*is == ' ' || *is == '\t' || *is == '\n' || *is == '\r')) { ++is; }
    for (const char* k = kw; *k != '\0'; ++k, ++is) {
        if (is >= end) { return false; }
        char c = *is;
        if (c >= 'A' && c <= 'Z') { c = static_cast<char>(c - 'A' + 'a'); }
        if (c != *k) { return false; }
    }
    return true;
}
} // namespace

int VTKAbstractReader::ReadCellsWithOffsetType(int& CellsNum, ArrayObject::Pointer& CellsId,
                                               ArrayObject::Pointer& CellsConnect) {
    int cellCount{0};
    int connSize{0};
    char buffer[256];
    if (!(this->Read(&cellCount) && this->Read(&connSize))) {
        igError("Error while reading cell array header.");
        return 0;
    }
    if (cellCount < 2) { return 1; }
    // 实测（VTK 5.1 新式布局）：头部第一个数是 **OFFSETS 个数**，而 offsets 数组有 ncells+1 项，
    // 因此 ncells = offsetsCount - 1；offsets 末项等于 connectivity 个数。
    CellsNum = cellCount - 1;
    if (!this->ReadString(buffer) ||                                                           // "offsets"
        (strcmp(this->LowerCase(buffer, 256), "offsets") != 0) || !this->ReadString(buffer)) { // datatype
        igError("Error reading cell array offset header.");
        return 0;
    }
    this->LowerCase(buffer, 256); // 数据类型 token（vtkIdType / vtktypeint64 / int …）
    const char* offsetsData = this->IS;

    // 稳健性：token 未必等于实际存储宽度（有的写出器标 vtkIdType 却存 4 字节）。
    // 先按框架既有的 4 字节 vtkidtype 路径读，再用「起点 0、单调不减、末位==连接数」校验；
    // 不通过则回到数据起点按 8 字节重读（文件在内存缓冲里，可直接回退）。
    int idWidth = 4;
    CellsId = this->ReadArray(buffer, cellCount, 1);
    if (CellsId == nullptr || !OffsetsLookSane(CellsId, CellsNum, connSize)) {
        idWidth = 8;
        this->IS = offsetsData;
        CellsId = this->ReadIdArrayWithElementSize(cellCount, idWidth);
        if (CellsId == nullptr || !OffsetsLookSane(CellsId, CellsNum, connSize)) {
            igError("Error reading cell array offset data (unsupported id width).");
            return 0;
        }
    }

    if (!this->ReadString(buffer) ||                                                              // "connectivity"
        (strcmp(this->LowerCase(buffer, 256), "connectivity") != 0) || !this->ReadString(buffer)) // datatype
    {
        igError("Error reading cell array connectivity header.");
        return 0;
    }
    this->LowerCase(buffer, 256);
    CellsConnect = (idWidth == 4) ? this->ReadArray(buffer, connSize, 1)
                                  : this->ReadIdArrayWithElementSize(connSize, idWidth);
    if (!CellsConnect) {
        igError("Error reading cell array connectivity data.");
        return 0;
    }
    this->UpdateReadProgress();
    return 1;
}

/** offsets 合理性校验：起点为 0、单调不减、末位等于连接数（连接数未知时只做前两项） */
bool VTKAbstractReader::OffsetsLookSane(ArrayObject::Pointer ids, int cellCount, int connSize) const {
    if (ids == nullptr || ids->GetNumberOfElements() < static_cast<IGsize>(cellCount) + 1) { return false; }
    double prev = ids->GetElementValue(0, 0);
    if (prev != 0.0) { return false; }
    for (int i = 1; i <= cellCount; ++i) {
        const double v = ids->GetElementValue(i, 0);
        if (v < prev) { return false; }
        prev = v;
    }
    if (connSize > 0 && static_cast<int>(prev) != connSize) { return false; }
    return true;
}

/** 按指定元素宽度（4 / 8 字节，大端）读取 id 数组 → IntArray（超出 int 范围的值按 int 截断） */
ArrayObject::Pointer VTKAbstractReader::ReadIdArrayWithElementSize(int count, int width) {
    if (count <= 0) { return IntArray::New(); }
    const size_t elemBytes = static_cast<size_t>(width);
    const size_t want = elemBytes * static_cast<size_t>(count);
    if (this->IS == nullptr || this->FILEEND == nullptr || static_cast<size_t>(this->FILEEND - this->IS) < want) {
        igError("Id array data truncated!");
        return nullptr;
    }
    std::vector<unsigned char> raw(want);
    if (!this->Read(reinterpret_cast<char*>(raw.data()), want)) { return nullptr; }

    auto arr = IntArray::New();
    arr->Resize(static_cast<IGsize>(count));
    int* out = arr->RawPointer();
    for (int i = 0; i < count; ++i) {
        const unsigned char* p = raw.data() + elemBytes * static_cast<size_t>(i);
        if (width == 8) {
            const unsigned long long v = (static_cast<unsigned long long>(p[0]) << 56) |
                                         (static_cast<unsigned long long>(p[1]) << 48) |
                                         (static_cast<unsigned long long>(p[2]) << 40) |
                                         (static_cast<unsigned long long>(p[3]) << 32) |
                                         (static_cast<unsigned long long>(p[4]) << 24) |
                                         (static_cast<unsigned long long>(p[5]) << 16) |
                                         (static_cast<unsigned long long>(p[6]) << 8) |
                                         static_cast<unsigned long long>(p[7]);
            out[i] = static_cast<int>(v);
        } else {
            const unsigned int v = (static_cast<unsigned int>(p[0]) << 24) |
                                   (static_cast<unsigned int>(p[1]) << 16) |
                                   (static_cast<unsigned int>(p[2]) << 8) | static_cast<unsigned int>(p[3]);
            out[i] = static_cast<int>(v);
        }
    }
    return arr;
}

int VTKAbstractReader::ReadCellsWithCellSizeType(int CellNum, int size, ArrayObject::Pointer& CellsId,
                                                 ArrayObject::Pointer& CellsConnect) {
    auto m_CellsId = IntArray::New();
    m_CellsId->Resize(static_cast<IGsize>(CellNum) + 1);
    int* ids = m_CellsId->RawPointer();
    auto m_CellsConnect = IntArray::New();
    const int connCapacity = (size > CellNum) ? (size - CellNum) : 0;
    m_CellsConnect->Resize(static_cast<IGsize>(connCapacity));
    int* conn = m_CellsConnect->RawPointer();
    int component, i, j;
    int index = 0;
    for (i = 0; i < CellNum; i++) {
        ids[i] = index;
        if (!this->Read(&component)) {
            igError("Cell id error!");
            return 0;
        }
        if (component < 0 || index + component > connCapacity) {
            igError("Cell size mismatch!");
            return 0;
        }
        for (j = 0; j < component; j++) {
            if (!this->Read(&conn[index + j])) {
                igError("Cell id error!");
                return 0;
            }
        }
        index += component;
    }
    ids[CellNum] = index;
    m_CellsConnect->Resize(static_cast<IGsize>(index)); // 截断到实际连接数
    this->UpdateReadProgress();
    CellsId = m_CellsId;
    CellsConnect = m_CellsConnect;
    return 1;
}

//------------------------------------------------------------------------------
// 经典格式「CELLS n size」的二进制版本：size 个大端 int32 连续存放（每单元 = 个数 + 点号）
int VTKAbstractReader::ReadCellsWithCellSizeTypeBinary(int CellNum, int size, ArrayObject::Pointer& CellsId,
                                                       ArrayObject::Pointer& CellsConnect) {
    if (CellNum <= 0 || size <= 0) {
        igError("Cell array header error!");
        return 0;
    }
    const size_t want = sizeof(int) * static_cast<size_t>(size);
    // FileReader::Read 在剩余数据不足时会静默截断，这里显式校验，避免用未初始化数据继续解析
    if (this->IS == nullptr || this->FILEEND == nullptr ||
        static_cast<size_t>(this->FILEEND - this->IS) < want) {
        igError("Cell connectivity data truncated!");
        return 0;
    }
    std::vector<int> buffer(static_cast<size_t>(size));
    if (!this->Read(reinterpret_cast<char*>(buffer.data()), want)) {
        igError("Cell connectivity data error!");
        return 0;
    }
    // 经典 VTK 二进制文件固定大端
    ByteSwap::Swap4BERange(buffer.data(), size);

    auto ids = IntArray::New();
    ids->Resize(static_cast<IGsize>(CellNum) + 1);
    int* idsPtr = ids->RawPointer();
    auto conn = IntArray::New();
    conn->Resize(static_cast<IGsize>(size));
    int* connPtr = conn->RawPointer();

    int pos = 0;
    int index = 0;
    for (int i = 0; i < CellNum; ++i) {
        if (pos >= size) {
            igError("Cell connectivity truncated!");
            return 0;
        }
        idsPtr[i] = index;
        const int component = buffer[static_cast<size_t>(pos++)];
        if (component < 0 || pos + component > size) {
            igError("Cell connectivity size mismatch!");
            return 0;
        }
        std::memcpy(connPtr + index, buffer.data() + pos, sizeof(int) * static_cast<size_t>(component));
        pos += component;
        index += component;
    }
    idsPtr[CellNum] = index;
    conn->Resize(static_cast<IGsize>(index)); // 截断到实际连接数
    this->UpdateReadProgress();
    CellsId = ids;
    CellsConnect = conn;
    return 1;
}
bool VTKAbstractReader::ReadUnstructuredGrid() {
    if (m_UnstructuredMesh == nullptr) { m_UnstructuredMesh = UnstructuredMesh::New(); }
    this->m_DataObjectType = IG_UNSTRUCTURED_MESH;
    // 分阶段计时：读取大网格时能直接看出时间花在哪一步（点/连接/建单元数组/属性）
    const auto elapsedMs = [](const std::chrono::steady_clock::time_point& t0) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    };
    double tPointsMs = 0.0, tCellsMs = 0.0, tBuildArraysMs = 0.0, tPointDataMs = 0.0, tCellDataMs = 0.0;
    char line[IGAME_CELL_MAX_SIZE];
    igIndex i = 0, PointsNum = 0, CellsNum = 0;
    igIndex npts, size = 0, ncells = 0;
    IntArray::Pointer Types;
    ArrayObject::Pointer CellsId;
    ArrayObject::Pointer CellsConnect;
    while (true) {
        if (!this->ReadString(line)) { break; }
        if (line[0] == '\0') {
            continue;
        } else if (!strncmp(this->LowerCase(line), "field", 5)) {
            this->ReadFieldData();
        } else if (!strncmp(line, "points", 6)) {
            if (!this->Read(&npts)) {
                igError("Cannot read number of points!");
                return false;
            }
            Points::Pointer points = Points::New();
            points->Reserve(npts);
            const auto tPoints = std::chrono::steady_clock::now();
            if (!this->ReadPointCoordinates(points, npts)) { return false; }
            tPointsMs += elapsedMs(tPoints);
            m_UnstructuredMesh->SetPoints(points);
        } else if (!strncmp(line, "cells", 5)) {
            // 两种文件类型都要区分：新式「CELLS n size」+ OFFSETS/CONNECTIVITY 段，或经典
            // 「CELLS n size」+ 连续数据（ASCII 文本 / 二进制大端 int32 块）。
            // 原先二进制分支无条件走 OFFSETS，遇到经典二进制文件会报错甚至越界崩溃。
            const auto tCells = std::chrono::steady_clock::now();
            const char* tmpIS = this->IS;
            // 新式布局是「CELLS <offsets个数> <连接个数>」后紧跟 OFFSETS 关键字，
            // 经典布局是「CELLS <单元数> <size>」后直接跟数据 —— 必须先读完两个数再判断。
            if (!(this->Read(&ncells) && this->Read(&size))) {
                igError("Cannot read cells!");
                return false;
            }
            const bool offsetStyle = PeekTokenIs(this->IS, this->FILEEND, "offsets");
            if (offsetStyle) {
                this->IS = tmpIS; // ReadCellsWithOffsetType 期望从 "CELLS" 之后开始
                if (!this->ReadCellsWithOffsetType(ncells, CellsId, CellsConnect)) { return false; }
            } else if (this->m_FileType == IGAME_BINARY) {
                // 「CELLS n size」头之后是行尾空白，再往后才是二进制数据块
                this->SkipNullData();
                if (!this->ReadCellsWithCellSizeTypeBinary(ncells, size, CellsId, CellsConnect)) { return false; }
            } else {
                if (!this->ReadCellsWithCellSizeType(ncells, size, CellsId, CellsConnect)) { return false; }
            }
            tCellsMs += elapsedMs(tCells);
            if (Types) {
                const auto tBuild = std::chrono::steady_clock::now();
                this->TransferVtkCellToiGameCell(CellsId, CellsConnect, Types);
                tBuildArraysMs += elapsedMs(tBuild);
            }
        } else if (!strncmp(line, "cell_types", 10)) {
            if (!this->Read(&ncells)) {
                igError("Cannot read cell types!");
                return false;
            }
            Types = DynamicCast<IntArray>(this->ReadArray("int", ncells, 1));
            // Update the dataset
            if (CellsId && CellsConnect) {
                const auto tBuild = std::chrono::steady_clock::now();
                this->TransferVtkCellToiGameCell(CellsId, CellsConnect, Types);
                tBuildArraysMs += elapsedMs(tBuild);
            }
        } else if (!strncmp(line, "cell_data", 9)) {
            if (!this->Read(&CellsNum)) {
                igError("Cannot read cell data!");
                return false;
            }
            if (ncells != CellsNum) {
                igError("Number of cells don't match!");
                return false;
            }
            const auto tCellData = std::chrono::steady_clock::now();
            this->ReadCellData(CellsNum);
            tCellDataMs += elapsedMs(tCellData);
            break; // out of this loop
        }

        else if (!strncmp(line, "point_data", 10)) {
            if (!this->Read(&PointsNum)) {
                igError("Cannot read point data!");
                return false;
            }
            if (npts != PointsNum) {
                igError("Number of points don't match!");
                return false;
            }
            const auto tPointData = std::chrono::steady_clock::now();
            this->ReadPointData(PointsNum);
            tPointDataMs += elapsedMs(tPointData);
            break; // out of this loop
        } else if (!strncmp(line, "metadata", 8)) {
            this->ProcessMetaData();
        } else {
            //igDebug("Unrecognized keyword: " << line);
            //return false;
        }
    }
    igDebug("VTK unstructured grid phases: points={} ms, cells={} ms, buildCellArrays={} ms, "
            "pointData={} ms, cellData={} ms",
            tPointsMs, tCellsMs, tBuildArraysMs, tPointDataMs, tCellDataMs);
    return true;
}

bool VTKAbstractReader::ReadSurfaceMesh() {
    if (m_SurfaceMesh == nullptr) { m_SurfaceMesh = SurfaceMesh::New(); }
    m_DataObjectType = IG_SURFACE_MESH;
    m_SurfaceMesh->SetFaces(iGame::CellArray::New());

    char line[IGAME_CELL_MAX_SIZE];
    char buffer[IGAME_CELL_MAX_SIZE];
    igIndex i = 0, PointsNum = 0, CellsNum = 0, size = 0;
    igIndex npts = 0, ncells = 0;
    igIndex offsetsSize = 0, connSize = 0;
    ArrayObject::Pointer CellsId;
    ArrayObject::Pointer CellsConnect;
    while (true) {
        if (!this->ReadString(line)) { break; }
        if (line[0] == '\0') {
            continue;
        } else if (!strncmp(this->LowerCase(line), "field", 5)) {
            this->ReadFieldData();
        } else if (!strncmp(line, "points", 6)) {
            if (!this->Read(&npts)) {
                igError("Cannot read number of points!");
                return false;
            }
            Points::Pointer points = Points::New();
            points->Reserve(npts);
            if (!this->ReadPointCoordinates(points, npts)) { return false; }
            m_SurfaceMesh->SetPoints(points);
        } 
          
        else if (!strncmp(line, "lines", 5)) {
            // 两种文件类型都区分「新式 OFFSETS/CONNECTIVITY」与「经典 n size + 连续数据」：
            // 原先二进制分支无条件走 OFFSETS（经典二进制 POLYDATA 会报错甚至越界崩溃），
            // ASCII 分支用 mAtoi 读 "OFFSETS" 还会越界访问数字表。
            const char* tmpIS = this->IS;
            // 新式布局是「<段名> <offsets个数> <连接个数>」后紧跟 OFFSETS 关键字，
            // 经典布局是「<段名> <单元数> <size>」后直接跟数据 —— 必须先读完两个数再判断。
            if (!(this->Read(&ncells) && this->Read(&size))) {
                igError("Cannot read cells!");
                return false;
            }
            if (PeekTokenIs(this->IS, this->FILEEND, "offsets")) {
                this->IS = tmpIS;
                if (!this->ReadCellsWithOffsetType(ncells, CellsId, CellsConnect)) { return false; }
            } else if (this->m_FileType == IGAME_BINARY) {
                this->SkipNullData();
                if (!this->ReadCellsWithCellSizeTypeBinary(ncells, size, CellsId, CellsConnect)) { return false; }
            } else {
                if (!this->ReadCellsWithCellSizeType(ncells, size, CellsId, CellsConnect)) { return false; }
            }
            m_SurfaceMesh->SetEdges(this->CreateCellArray(CellsId, CellsConnect));
        }
        
        
        else if (!strncmp(line, "polygons", 8)) {
            // 两种文件类型都区分「新式 OFFSETS/CONNECTIVITY」与「经典 n size + 连续数据」：
            // 原先二进制分支无条件走 OFFSETS（经典二进制 POLYDATA 会报错甚至越界崩溃），
            // ASCII 分支用 mAtoi 读 "OFFSETS" 还会越界访问数字表。
            const char* tmpIS = this->IS;
            // 新式布局是「<段名> <offsets个数> <连接个数>」后紧跟 OFFSETS 关键字，
            // 经典布局是「<段名> <单元数> <size>」后直接跟数据 —— 必须先读完两个数再判断。
            if (!(this->Read(&ncells) && this->Read(&size))) {
                igError("Cannot read cells!");
                return false;
            }
            if (PeekTokenIs(this->IS, this->FILEEND, "offsets")) {
                this->IS = tmpIS;
                if (!this->ReadCellsWithOffsetType(ncells, CellsId, CellsConnect)) { return false; }
            } else if (this->m_FileType == IGAME_BINARY) {
                this->SkipNullData();
                if (!this->ReadCellsWithCellSizeTypeBinary(ncells, size, CellsId, CellsConnect)) { return false; }
            } else {
                if (!this->ReadCellsWithCellSizeType(ncells, size, CellsId, CellsConnect)) { return false; }
            }
            m_SurfaceMesh->SetFaces(this->CreateCellArray(CellsId, CellsConnect));
        } else if (!strncmp(line, "cell_data", 9)) {
            if (!this->Read(&CellsNum)) {
                igError("Cannot read cell data!");
                return false;
            }
            if (ncells != CellsNum) {
                igError("Number of cells don't match!");
                return false;
            }
            this->ReadCellData(ncells);
            break; // out of this loop
        } else if (!strncmp(line, "point_data", 10)) {
            if (!this->Read(&PointsNum)) {
                igError("Cannot read point data!");
                return false;
            }
            if (npts != PointsNum) {
                igError("Number of points don't match!");
                return false;
            }
            this->ReadPointData(PointsNum);
            break; // out of this loop
        } else if (!strncmp(line, "metadata", 8)) {
            this->ProcessMetaData();
        } else {
            //igDebug("Unrecognized keyword: " << line);
            //return false;
        }
    }
    return true;
}
bool VTKAbstractReader::ReadStructuredGrid() {
    if (m_StructuredMesh == nullptr) { m_StructuredMesh = StructuredMesh::New(); }
    m_DataObjectType = IG_STRUCTURED_MESH;
    char line[IGAME_CELL_MAX_SIZE];
    char buffer[IGAME_CELL_MAX_SIZE];
    igIndex size[3];
    double origin[3];
    double aspectRatio[3];
    igIndex npts = 0;
    igIndex ncells = 0;

    while (true) {
        if (!this->ReadString(line)) { break; }
        if (line[0] == '\0') { continue; }
        if (!strncmp(this->LowerCase(line), "dimensions", 10)) {
            if (!this->Read(&size[0]) || !this->Read(&size[1]) || !this->Read(&size[2])) {
                igError("Read dimension error!");
                return false;
            }
            if (size[2] == 0) { size[2] = 1; }
            m_StructuredMesh->SetDimensionSize(size);
        } else if (!strncmp(line, "aspect_ratio", 12)) {
            if (!this->Read(&aspectRatio[0]) || !this->Read(&aspectRatio[1]) || !this->Read(&aspectRatio[2])) {
                igError("Read extend error!");
                return false;
            }
        } else if (!strncmp(line, "origin", 6)) {
            if (!this->Read(&origin[0]) || !this->Read(&origin[1]) || !this->Read(&origin[2])) {
                igError("Read extend error!");
                return false;
            }
        } else if (!strncmp(line, "points", 6)) {
            if (!this->Read(&npts)) {
                igError("Cannot read number of points!");
                return false;
            }
            if (size[0] * size[1] * size[2] != npts) {
                igError("Number of points don't match!");
                return false;
            }
            Points::Pointer points = Points::New();
            if (!this->ReadPointCoordinates(points, npts)) { return false; }
            m_StructuredMesh->SetPoints(points);
        } else if (!strncmp(line, "cell_data", 9)) {
            if (!this->Read(&ncells)) {
                igError("Cannot read cell data!");
                return false;
            }
            if ((size[2] == 1 && (ncells != (size[0] - 1) * (size[1] - 1))) ||
                (size[2] != 1 && (ncells != (size[0] - 1) * (size[1] - 1) * (size[2] - 1)))) {
                igError("Number of cells don't match!");
                return false;
            }
            this->ReadCellData(ncells);
            break; // out of this loop
        } else if (!strncmp(line, "point_data", 10)) {
            if (!this->Read(&npts)) {
                igError("Cannot read point data!");
                return false;
            }
            if (size[0] * size[1] * size[2] != npts) {
                igError("Number of points don't match!");
                return false;
            }
            this->ReadPointData(npts);
            break; // out of this loop
        } else if (!strncmp(line, "metadata", 8)) {
            this->ProcessMetaData();
        } else {
            //igDebug("Unrecognized keyword: " << line);
            //return false;
        }
    }
    if (m_StructuredMesh->GetPoints() == nullptr || m_StructuredMesh->GetPoints()->GetNumberOfPoints() == 0) {
        igIndex i, j, k;
        Point p;
        Points::Pointer points = Points::New();
        points->Reserve(npts);
        p[2] = origin[2];
        for (k = 0; k < size[2]; ++k) {
            p[1] = origin[1];
            for (j = 0; j < size[1]; ++j) {
                p[0] = origin[0];
                for (i = 0; i < size[0]; ++i) {
                    points->AddPoint(p);
                    p[0] += aspectRatio[0];
                }
                p[1] += aspectRatio[1];
            }
            p[2] += aspectRatio[2];
        }
        m_StructuredMesh->SetPoints(points);
    }
    m_StructuredMesh->GenStructuredCellConnectivities();
    return true;
}
CellArray::Pointer VTKAbstractReader::CreateCellArray(ArrayObject::Pointer CellsID, ArrayObject::Pointer CellsConnect) {
    // Each POLYGONS/LINES section owns its connectivity. Reset removes the
    // initial zero offset (breaking mixed polygons), and also overwrites any
    // preceding section that already holds this same CellArray.
    m_CellArray = CellArray::New();

    if (CellsID == nullptr || CellsConnect == nullptr || CellsID->GetNumberOfElements() < 2) {
        igError("Invalid cell arrays while creating CellArray.");
        return m_CellArray;
    }

    int CellNum = CellsID->GetNumberOfElements() - 1;
    const int connectCount = static_cast<int>(CellsConnect->GetNumberOfElements());
    int skippedCells = 0;

    for (int i = 0; i < CellNum; i++) {
        const int st = static_cast<int>(CellsID->GetValue(i));
        const int ed = static_cast<int>(CellsID->GetValue(i + 1));
        const int size = ed - st;
        if (st < 0 || ed < st || ed > connectCount || size <= 0) {
            skippedCells++;
            continue;
        }

        std::vector<igIndex> vhs(size);
        for (int j = 0; j < size; j++) { vhs[j] = static_cast<igIndex>(CellsConnect->GetValue(st + j)); }
        m_CellArray->AddCellIds(vhs.data(), size);
    }

    if (skippedCells > 0) { igDebug("CreateCellArray skipped invalid cells: {}", skippedCells); }
    return m_CellArray;
}

const void VTKAbstractReader::TransferVtkCellToiGameCell(ArrayObject::Pointer CellsID,
                                                         ArrayObject::Pointer CellsConnect,
                                                         IntArray::Pointer VtkCellsType) {
    DataObject::Pointer Mesh = m_UnstructuredMesh;
    this->m_DataObjectType = IG_UNSTRUCTURED_MESH;
    TransferVtkCellToiGameCell(Mesh, CellsID, CellsConnect, VtkCellsType);
    m_UnstructuredMesh = DynamicCast<UnstructuredMesh>(Mesh);
    this->UpdateReadProgress();
}

void VTKAbstractReader::TransferVtkCellToiGameCell(DataObject::Pointer& _mesh, ArrayObject::Pointer CellsID,
                                                   ArrayObject::Pointer CellsConnect,
                                                   ArrayObject::Pointer VtkCellsType) {
    TransferVtkCellToiGameCell(_mesh, CellsID, CellsConnect, VtkCellsType, nullptr, nullptr, nullptr, nullptr);
}

void VTKAbstractReader::TransferVtkCellToiGameCell(DataObject::Pointer& _mesh, ArrayObject::Pointer CellsID,
                                                   ArrayObject::Pointer CellsConnect,
                                                   ArrayObject::Pointer VtkCellsType,
                                                   ArrayObject::Pointer FacesConnect,
                                                   ArrayObject::Pointer FacesOffset,
                                                   ArrayObject::Pointer PolyToFaces,
                                                   ArrayObject::Pointer PolysOffset) {
    if (CellsID == nullptr || CellsConnect == nullptr || VtkCellsType == nullptr) {
        igError("Invalid input arrays in TransferVtkCellToiGameCell.");
        return;
    }
    UnstructuredMesh::Pointer mesh = _mesh == nullptr ? UnstructuredMesh::New() : DynamicCast<UnstructuredMesh>(_mesh);
    const int CellNum = static_cast<int>(VtkCellsType->GetNumberOfElements());
    const int offsetsCount = static_cast<int>(CellsID->GetNumberOfElements());
    const int connectCount = static_cast<int>(CellsConnect->GetNumberOfElements());
    int skippedCells = 0;
    int normalizedCells = 0;
    bool onlyVertexCells = CellNum > 0 && mesh->GetNumberOfCells() == 0;

    // 快路径：三个输入数组通常是 IntArray，取一次原始指针即可，避免每单元/每点一次虚函数调用
    const int* idPtr = nullptr;
    const int* connPtr = nullptr;
    const int* typePtr = nullptr;
    if (auto a = DynamicCast<IntArray>(CellsID); !a.IsNull()) { idPtr = a->RawPointer(); }
    if (auto a = DynamicCast<IntArray>(CellsConnect); !a.IsNull()) { connPtr = a->RawPointer(); }
    if (auto a = DynamicCast<IntArray>(VtkCellsType); !a.IsNull()) { typePtr = a->RawPointer(); }
    const auto idAt = [&](int k) { return idPtr != nullptr ? idPtr[k] : static_cast<int>(CellsID->GetValue(k)); };
    const auto connAt = [&](int k) {
        return connPtr != nullptr ? connPtr[k] : static_cast<int>(CellsConnect->GetValue(k));
    };
    const auto typeAt = [&](int k) {
        return typePtr != nullptr ? typePtr[k] : static_cast<int>(VtkCellsType->GetValue(k));
    };

    // 复用同一块缓冲：原先每个单元都构造一次 std::vector（百万单元即百万次堆分配）
    std::vector<igIndex> vhs;

    for (int i = 0; i < CellNum; i++) {
        if (i + 1 >= offsetsCount) {
            skippedCells++;
            continue;
        }

        const int st = idAt(i);
        const int ed = idAt(i + 1);
        int size = ed - st;
        if (st < 0 || ed < st || ed > connectCount || size <= 0) {
            skippedCells++;
            continue;
        }

        const VTKTYPE declaredType = (VTKTYPE) typeAt(i);
        onlyVertexCells &= declaredType == VERTEX;
        VTKTYPE type = NormalizeVtkCellType(declaredType, size);
        if (type != declaredType) normalizedCells++;
        if (!IsVtkCellSizeValid(type, size)) {
            skippedCells++;
            continue;
        }

        vhs.resize(static_cast<size_t>(size));

        if (type != POLYHEDRON) {
            for (int j = 0; j < size; j++) { vhs[j] = static_cast<igIndex>(connAt(st + j)); }
        } else {
            // for celltype == 42
            bool isValid = (FacesConnect && FacesConnect->GetNumberOfElements() > 0 && PolysOffset &&
                            PolysOffset->GetNumberOfElements() > 0);
            if (!isValid) {
                igError("no faces/face_connectivity/faceoffsets/polyhedron_offsets data array found in POLYHEDRON "
                        "cell");
                return;
            }

            if (PolyToFaces && PolyToFaces->GetNumberOfElements() > 0) {
                // >= 2.3 version
                isValid = (FacesOffset && FacesOffset->GetNumberOfElements() > 0);
                if (!isValid) {
                    igError("no face_offsets data array found in POLYHEDRON cell (>=2.3 version)");
                    return;
                }

                int fSt = static_cast<int>(PolysOffset->GetValue(i));
                int fEd = static_cast<int>(PolysOffset->GetValue(i + 1));
                int numFaces = fEd - fSt;

                // compute the vhs size in advance
                int totalSize = 1; // numFaces
                for (int f = fSt; f < fEd; f++) {
                    int faceId = PolyToFaces->GetValue(f);
                    int pSt = static_cast<int>(FacesOffset->GetValue(faceId));
                    int pEd = static_cast<int>(FacesOffset->GetValue(faceId + 1));
                    totalSize += 1 + (pEd - pSt); // 1(fSize) + facePointCount
                }
                vhs.resize(totalSize);

                // make sure the vhs structure is: [numFaces, fSize0, v0..vN, fSize1, v0..vM, ...]
                int idx = 0;
                vhs[idx++] = numFaces;
                for (int f = fSt; f < fEd; f++) {
                    int faceId = PolyToFaces->GetValue(f);
                    if (faceId + 1 >= FacesOffset->GetNumberOfElements()) {
                        skippedCells++;
                        continue;
                    }
                    int pSt = static_cast<int>(FacesOffset->GetValue(faceId));
                    int pEd = static_cast<int>(FacesOffset->GetValue(faceId + 1));
                    int numPts = pEd - pSt;
                    vhs[idx++] = numPts;
                    for (int k = pSt; k < pSt + numPts; k++) {
                        vhs[idx++] = static_cast<igIndex>(FacesConnect->GetValue(k));
                    }
                }
                size = totalSize;
            } else {
                // < 2.3 version
                int fSt = static_cast<int>(PolysOffset->GetValue(i));
                int fEd = static_cast<int>(PolysOffset->GetValue(i + 1));
                int totalSize = fEd - fSt;
                vhs.resize(totalSize);
                for (int k = 0; k < totalSize; k++) { vhs[k] = static_cast<igIndex>(FacesConnect->GetValue(fSt + k)); }
                size = totalSize;
            }
        }

        switch (type) {
            case iGame::VTKAbstractReader::T0:
                break;
            case iGame::VTKAbstractReader::VERTEX:
                mesh->AddCell(vhs.data(), size, IG_VERTEX);
                break;
            case iGame::VTKAbstractReader::POLYVERTEX:
                break;
            case iGame::VTKAbstractReader::LINE:
                mesh->AddCell(vhs.data(), size, IG_LINE);
                break;
            case iGame::VTKAbstractReader::POLYLINE:
                mesh->AddCell(vhs.data(), size, IG_POLY_LINE);
                break;
            case iGame::VTKAbstractReader::TRIANGLE:
                mesh->AddCell(vhs.data(), size, IG_TRIANGLE);
                break;
            case iGame::VTKAbstractReader::TRIANGLESTRIP:
                break;
            case iGame::VTKAbstractReader::POLYGON:
                mesh->AddCell(vhs.data(), size, IG_POLYGON);
                break;
            case iGame::VTKAbstractReader::PIXEL:
                mesh->AddCell(vhs.data(), size, IG_QUAD);
                break;
            case iGame::VTKAbstractReader::QUAD:
                mesh->AddCell(vhs.data(), size, IG_QUAD);
                break;
            case iGame::VTKAbstractReader::TETRA:
                mesh->AddCell(vhs.data(), size, IG_TETRA);
                break;
            case iGame::VTKAbstractReader::VOXEL: {
                // VTK_VOXEL (cell type 11) is topologically a hexahedron whose node
                // ordering differs from VTK_HEXAHEDRON. Reorder the 8 point ids into
                // iGame hexahedron ordering and store it as IG_HEXAHEDRON so that
                // rendering, surface extraction and the IGC codec all handle it.
                static constexpr igIndex VoxelToHexahedron[8] = {0, 1, 3, 2, 4, 5, 7, 6};
                igIndex hexVhs[8];
                for (int j = 0; j < 8; ++j) { hexVhs[j] = vhs[VoxelToHexahedron[j]]; }
                mesh->AddCell(hexVhs, size, IG_HEXAHEDRON);
            } break;
            case iGame::VTKAbstractReader::HEXAHEDRON:
                mesh->AddCell(vhs.data(), size, IG_HEXAHEDRON);
                break;
            case iGame::VTKAbstractReader::WEDGE:
                mesh->AddCell(vhs.data(), size, IG_PRISM);
                break;
            case iGame::VTKAbstractReader::PYRAMID:
                mesh->AddCell(vhs.data(), size, IG_PYRAMID);
                break;
            case iGame::VTKAbstractReader::PENTAGONAL_PRISM:
                break;
            case iGame::VTKAbstractReader::HEXAGONAL_PRISM:
                break;
            case iGame::VTKAbstractReader::T17:
                break;
            case iGame::VTKAbstractReader::T18:
                break;
            case iGame::VTKAbstractReader::T19:
                break;
            case iGame::VTKAbstractReader::T20:
                break;
            case iGame::VTKAbstractReader::QUADRATIC_EDGE:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_EDGE);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_TRIANGLE:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_TRIANGLE);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_QUAD:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_QUAD);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_TETRA:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_TETRA);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_HEXAHEDRON:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_HEXAHEDRON);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_WEDGE:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_PRISM);
                break;
            case iGame::VTKAbstractReader::QUADRATIC_PYRAMID:
                mesh->AddCell(vhs.data(), size, IG_QUADRATIC_PYRAMID);
                break;
            case iGame::VTKAbstractReader::BIQUADRATIC_QUAD:
                break;
            case iGame::VTKAbstractReader::TRIQUADRATIC_HEXAHEDRON:
                break;
            case iGame::VTKAbstractReader::QUADRATIC_LINEAR_QUAD:
                break;
            case iGame::VTKAbstractReader::QUADRATIC_LINEAR_WEDGE:
                break;
            case iGame::VTKAbstractReader::BIQUADRATIC_QUADRATIC_WEDGE:
                break;
            case iGame::VTKAbstractReader::BIQUADRATIC_QUADRATIC_HEXAHEDRON:
                break;
            case iGame::VTKAbstractReader::BIQUADRATIC_TRIANGLE:
                break;
            case iGame::VTKAbstractReader::CUBIC_LINE:
                break;
            case iGame::VTKAbstractReader::QUADRATIC_POLYGON:
                break;
            case iGame::VTKAbstractReader::TRIQUADRATIC_PYRAMID:
                break;
            case iGame::VTKAbstractReader::POLYHEDRON: //42
            {
                mesh->AddCell(vhs.data(), size, IG_POLYHEDRON);
            } break;
            case iGame::VTKAbstractReader::LAGRANGE_TETRAHEDRON:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_TETRAHEDRON);
                break;
            case iGame::VTKAbstractReader::LAGRANGE_HEXAHEDRON:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_HEXAHEDRON);
                break;
            case iGame::VTKAbstractReader::LAGRANGE_PRISM:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_PRISM);
                break;
            case iGame::VTKAbstractReader::LAGRANGE_PYRAMID:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_PYRAMID);
                break;
            case iGame::VTKAbstractReader::LAGRANGE_QUADRILATERAL:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_QUADRILATERAL);
                break;
            case iGame::VTKAbstractReader::LAGRANGE_TRIANGLE:
                mesh->AddCell(vhs.data(), size, IG_LAGRANGE_TRIANGLE);
                break;
            default:
                break;
        }
    }
    if (normalizedCells > 0) {
        igDebug("TransferVtkCellToiGameCell normalized mislabeled quadratic cells: {}", normalizedCells);
    }
    if (skippedCells > 0) { igDebug("TransferVtkCellToiGameCell skipped invalid cells: {}", skippedCells); }
    // A vertex-only grid has no surface to extract. Preserve its cells and
    // attributes, and draw the original points instead of an empty shell.
    if (onlyVertexCells && mesh->GetNumberOfCells() == CellNum) {
        mesh->SetShellRenderingOption(false);
        mesh->SetViewStyle(IG_POINTS);
    }
    _mesh = mesh;
}


IGAME_NAMESPACE_END
