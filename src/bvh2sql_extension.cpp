#define DUCKDB_EXTENSION_MAIN

#include "bvh2sql_extension.hpp"
#include "duckdb.hpp"
#include "duckdb/common/exception.hpp"
#include "duckdb/function/table_function.hpp"
#include "duckdb/parser/parsed_data/create_table_function_info.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace bvh_absolute {

// =============================================================================
// Math Utilities - 行列計算
// =============================================================================

struct Matrix4x4 {
    double m[4][4];
    
    Matrix4x4() {
        // 単位行列
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
    // 行列の乗算
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0.0;
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }
    
    // 平行移動行列を作成
    static Matrix4x4 translation(double x, double y, double z) {
        Matrix4x4 mat;
        mat.m[0][3] = x;
        mat.m[1][3] = y;
        mat.m[2][3] = z;
        return mat;
    }
    
    // X軸回転行列
    static Matrix4x4 rotationX(double degrees) {
        Matrix4x4 mat;
        double rad = degrees * M_PI / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);
        mat.m[1][1] = c;
        mat.m[1][2] = -s;
        mat.m[2][1] = s;
        mat.m[2][2] = c;
        return mat;
    }
    
    // Y軸回転行列
    static Matrix4x4 rotationY(double degrees) {
        Matrix4x4 mat;
        double rad = degrees * M_PI / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);
        mat.m[0][0] = c;
        mat.m[0][2] = s;
        mat.m[2][0] = -s;
        mat.m[2][2] = c;
        return mat;
    }
    
    // Z軸回転行列
    static Matrix4x4 rotationZ(double degrees) {
        Matrix4x4 mat;
        double rad = degrees * M_PI / 180.0;
        double c = std::cos(rad);
        double s = std::sin(rad);
        mat.m[0][0] = c;
        mat.m[0][1] = -s;
        mat.m[1][0] = s;
        mat.m[1][1] = c;
        return mat;
    }
    
    // 位置ベクトルを変換
    void transformPoint(double& x, double& y, double& z) const {
        double tx = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3];
        double ty = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3];
        double tz = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3];
        x = tx;
        y = ty;
        z = tz;
    }
};

// =============================================================================
// BVH Data Structures
// =============================================================================

enum class ChannelType {
    XPOSITION, YPOSITION, ZPOSITION,
    XROTATION, YROTATION, ZROTATION
};

struct Joint {
    std::string name;
    Joint* parent;
    std::vector<std::unique_ptr<Joint>> children;
    
    double offset_x, offset_y, offset_z;
    std::vector<ChannelType> channels;
    int channel_start_index;
    
    Joint(const std::string& n, Joint* p = nullptr)
        : name(n), parent(p), offset_x(0), offset_y(0), offset_z(0), 
          channel_start_index(-1) {}
};

struct BVHData {
    std::unique_ptr<Joint> root;
    std::vector<Joint*> joint_list;
    int num_frames;
    double frame_time;
    std::vector<std::vector<double>> motion_data;
    
    BVHData() : num_frames(0), frame_time(0.0) {}
};

inline std::string channelTypeToString(ChannelType type) {
    switch (type) {
        case ChannelType::XPOSITION: return "Xposition";
        case ChannelType::YPOSITION: return "Yposition";
        case ChannelType::ZPOSITION: return "Zposition";
        case ChannelType::XROTATION: return "Xrotation";
        case ChannelType::YROTATION: return "Yrotation";
        case ChannelType::ZROTATION: return "Zrotation";
        default: return "Unknown";
    }
}

inline ChannelType stringToChannelType(const std::string& str) {
    if (str == "Xposition") return ChannelType::XPOSITION;
    if (str == "Yposition") return ChannelType::YPOSITION;
    if (str == "Zposition") return ChannelType::ZPOSITION;
    if (str == "Xrotation") return ChannelType::XROTATION;
    if (str == "Yrotation") return ChannelType::YROTATION;
    if (str == "Zrotation") return ChannelType::ZROTATION;
    return ChannelType::XPOSITION;
}

// =============================================================================
// BVH Parser
// =============================================================================

class BVHParser {
public:
    bool parse(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            error_ = "Failed to open file: " + filename;
            return false;
        }
        
        if (!parseHierarchy(file)) return false;
        if (!parseMotion(file)) return false;
        
        return true;
    }
    
    const BVHData& getData() const { return data_; }
    const std::string& getError() const { return error_; }
    
private:
    BVHData data_;
    std::string error_;
    
    // CRLF/LFの両方に対応したtrim関数
    std::string trim(const std::string& str) {
        auto start = std::find_if_not(str.begin(), str.end(), [](unsigned char ch) {
            return std::isspace(ch) || ch == '\r' || ch == '\n' || ch == '\t';
        });
        auto end = std::find_if_not(str.rbegin(), str.rend(), [](unsigned char ch) {
            return std::isspace(ch) || ch == '\r' || ch == '\n' || ch == '\t';
        }).base();
        return (start < end) ? std::string(start, end) : std::string();
    }
    
    std::vector<std::string> split(const std::string& str) {
        std::vector<std::string> result;
        std::istringstream iss(str);
        std::string token;
        while (iss >> token) {
            // トークンが空でないことを確認
            if (!token.empty()) {
                result.push_back(token);
            }
        }
        return result;
    }
    
    bool parseHierarchy(std::istream& stream) {
        std::string line;
        if (!std::getline(stream, line) || trim(line) != "HIERARCHY") {
            error_ = "Expected HIERARCHY";
            return false;
        }
        data_.root = std::unique_ptr<Joint>(parseJoint(stream, nullptr));
        return data_.root != nullptr;
    }
    
    Joint* parseJoint(std::istream& stream, Joint* parent) {
        std::string line;
        if (!std::getline(stream, line)) return nullptr;
        
        auto tokens = split(trim(line));
        if (tokens.size() < 2) return nullptr;
        
        // ROOT or JOINT
        std::string joint_type = tokens[0];
        Joint* joint = new Joint(tokens[1], parent);
        data_.joint_list.push_back(joint);
        
        if (!std::getline(stream, line) || trim(line) != "{") {
            error_ = "Expected { after " + joint->name;
            return nullptr;
        }
        
        while (std::getline(stream, line)) {
            auto trimmed = trim(line);
            
            // この階層の終わり
            if (trimmed == "}") {
                return joint;
            }
            
            tokens = split(trimmed);
            if (tokens.empty()) continue;
            
            if (tokens[0] == "OFFSET" && tokens.size() >= 4) {
                joint->offset_x = std::stod(tokens[1]);
                joint->offset_y = std::stod(tokens[2]);
                joint->offset_z = std::stod(tokens[3]);
            }
            else if (tokens[0] == "CHANNELS" && tokens.size() >= 2) {
                int num_channels = std::stoi(tokens[1]);
                joint->channel_start_index = getCurrentChannelCount();
                for (int i = 0; i < num_channels && i + 2 < (int)tokens.size(); ++i) {
                    joint->channels.push_back(stringToChannelType(tokens[i + 2]));
                }
            }
            else if (tokens[0] == "JOINT") {
                // 再帰的に子ジョイントをパース
                Joint* child = parseJoint(stream, joint);
                if (child) {
                    joint->children.push_back(std::unique_ptr<Joint>(child));
                }
            }
            else if (tokens[0] == "End" && tokens.size() >= 2 && tokens[1] == "Site") {
                parseEndSite(stream, joint);
            }
        }
        
        error_ = "Unexpected end of file in joint: " + joint->name;
        return nullptr;
    }
    
    void parseEndSite(std::istream& stream, Joint* parent) {
        std::string line;
        
        // { を読み飛ばす
        if (!std::getline(stream, line)) return;
        
        // End Site の内容を読む
        while (std::getline(stream, line)) {
            if (trim(line) == "}") {
                return;
            }
        }
    }
    
    int getCurrentChannelCount() const {
        int count = 0;
        for (const auto& joint : data_.joint_list) {
            count += joint->channels.size();
        }
        return count;
    }
    
    bool parseMotion(std::istream& stream) {
        std::string line;
        
        // MOTIONを探す
        while (std::getline(stream, line)) {
            if (trim(line) == "MOTION") {
                break;
            }
        }
        
        if (stream.eof()) {
            error_ = "Expected MOTION section";
            return false;
        }
        
        if (!std::getline(stream, line)) return false;
        auto tokens = split(trim(line));
        if (tokens.size() < 2 || tokens[0] != "Frames:") {
            error_ = "Expected Frames:";
            return false;
        }
        data_.num_frames = std::stoi(tokens[1]);
        
        if (!std::getline(stream, line)) return false;
        tokens = split(trim(line));
        if (tokens.size() < 3 || tokens[0] != "Frame" || tokens[1] != "Time:") {
            error_ = "Expected Frame Time:";
            return false;
        }
        // 先頭にピリオドがある場合に対応 (.0083333 -> 0.0083333)
        std::string time_str = tokens[2];
        if (!time_str.empty() && time_str[0] == '.') {
            time_str = "0" + time_str;
        }
        data_.frame_time = std::stod(time_str);
        
        data_.motion_data.resize(data_.num_frames);
        for (int i = 0; i < data_.num_frames; ++i) {
            if (!std::getline(stream, line)) {
                error_ = "Unexpected end of file";
                return false;
            }
            tokens = split(trim(line));
            for (const auto& token : tokens) {
                data_.motion_data[i].push_back(std::stod(token));
            }
        }
        
        return true;
    }
};

// =============================================================================
// Absolute Position Calculator
// =============================================================================

struct AbsoluteTransform {
    double world_x, world_y, world_z;
    double rot_x, rot_y, rot_z;
};

class AbsolutePositionCalculator {
public:
    AbsolutePositionCalculator(const BVHData& data) : data_(data) {}
    
    std::map<std::string, AbsoluteTransform> calculateFrame(int frame_id) {
        std::map<std::string, AbsoluteTransform> results;
        Matrix4x4 identity;
        
        if (data_.root) {
            calculateJointRecursive(data_.root.get(), frame_id, identity, results);
        }
        
        return results;
    }
    
private:
    const BVHData& data_;
    
    void calculateJointRecursive(Joint* joint, int frame_id, 
                                 const Matrix4x4& parent_transform,
                                 std::map<std::string, AbsoluteTransform>& results) {
        // ローカル変換行列を作成
        Matrix4x4 local_transform;
        
        // オフセットを適用
        local_transform = Matrix4x4::translation(joint->offset_x, 
                                                  joint->offset_y, 
                                                  joint->offset_z);
        
        // チャンネルデータを取得して適用
        AbsoluteTransform transform;
        double pos_x = 0, pos_y = 0, pos_z = 0;
        double rot_x = 0, rot_y = 0, rot_z = 0;
        
        for (size_t i = 0; i < joint->channels.size(); ++i) {
            int data_index = joint->channel_start_index + i;
            double value = data_.motion_data[frame_id][data_index];
            
            switch (joint->channels[i]) {
                case ChannelType::XPOSITION: pos_x = value; break;
                case ChannelType::YPOSITION: pos_y = value; break;
                case ChannelType::ZPOSITION: pos_z = value; break;
                case ChannelType::XROTATION: rot_x = value; break;
                case ChannelType::YROTATION: rot_y = value; break;
                case ChannelType::ZROTATION: rot_z = value; break;
            }
        }
        
        // 位置チャンネルがあれば追加の平行移動
        if (pos_x != 0 || pos_y != 0 || pos_z != 0) {
            local_transform = local_transform * Matrix4x4::translation(pos_x, pos_y, pos_z);
        }
        
        // 回転を適用（BVHの標準的な順序: Z, X, Y）
        if (rot_z != 0) local_transform = local_transform * Matrix4x4::rotationZ(rot_z);
        if (rot_x != 0) local_transform = local_transform * Matrix4x4::rotationX(rot_x);
        if (rot_y != 0) local_transform = local_transform * Matrix4x4::rotationY(rot_y);
        
        // ワールド変換 = 親の変換 × ローカル変換
        Matrix4x4 world_transform = parent_transform * local_transform;
        
        // ワールド座標を取得（原点を変換）
        double world_x = 0, world_y = 0, world_z = 0;
        world_transform.transformPoint(world_x, world_y, world_z);
        
        // 結果を保存
        transform.world_x = world_x;
        transform.world_y = world_y;
        transform.world_z = world_z;
        transform.rot_x = rot_x;
        transform.rot_y = rot_y;
        transform.rot_z = rot_z;
        
        results[joint->name] = transform;
        
        // 子ジョイントを再帰的に処理
        for (const auto& child : joint->children) {
            calculateJointRecursive(child.get(), frame_id, world_transform, results);
        }
    }
};

// =============================================================================
// DuckDB Table Function
// =============================================================================

static std::map<std::string, std::shared_ptr<BVHData>> bvh_cache;

static std::shared_ptr<BVHData> getOrParseBVH(const std::string& filename) {
    auto it = bvh_cache.find(filename);
    if (it != bvh_cache.end()) {
        return it->second;
    }
    
    BVHParser parser;
    if (!parser.parse(filename)) {
        throw duckdb::IOException("Failed to parse BVH file: " + parser.getError());
    }
    
    // BVHDataを新しく作成してmoveで転送
    auto data = std::make_shared<BVHData>();
    *data = std::move(const_cast<BVHData&>(parser.getData()));
    bvh_cache[filename] = data;
    return data;
}

struct AbsolutePositionBindData : public duckdb::TableFunctionData {
    std::string filename;
    std::shared_ptr<BVHData> bvh_data;
};

struct AbsolutePositionGlobalState : public duckdb::GlobalTableFunctionState {
    duckdb::idx_t current_row;
    AbsolutePositionGlobalState() : current_row(0) {}
    duckdb::idx_t MaxThreads() const override { return 1; }
};

static duckdb::unique_ptr<duckdb::FunctionData> AbsolutePositionBind(
    duckdb::ClientContext &context, 
    duckdb::TableFunctionBindInput &input,
    duckdb::vector<duckdb::LogicalType> &return_types, 
    duckdb::vector<duckdb::string> &names) {
    
    auto result = duckdb::make_uniq<AbsolutePositionBindData>();
    
    if (input.inputs.empty()) {
        throw duckdb::InvalidInputException("bvh_absolute_positions requires a filename argument");
    }
    
    result->filename = input.inputs[0].ToString();
    result->bvh_data = getOrParseBVH(result->filename);
    
    names.emplace_back("frame_id");
    return_types.emplace_back(duckdb::LogicalType::INTEGER);
    
    names.emplace_back("time");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("joint_name");
    return_types.emplace_back(duckdb::LogicalType::VARCHAR);
    
    names.emplace_back("world_x");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("world_y");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("world_z");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("rot_x");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("rot_y");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    names.emplace_back("rot_z");
    return_types.emplace_back(duckdb::LogicalType::DOUBLE);
    
    return std::move(result);
}

static duckdb::unique_ptr<duckdb::GlobalTableFunctionState> AbsolutePositionInit(
    duckdb::ClientContext &context, 
    duckdb::TableFunctionInitInput &input) {
    return duckdb::make_uniq<AbsolutePositionGlobalState>();
}

static void AbsolutePositionFunction(
    duckdb::ClientContext &context, 
    duckdb::TableFunctionInput &data_p, 
    duckdb::DataChunk &output) {
    
    auto &bind_data = data_p.bind_data->Cast<AbsolutePositionBindData>();
    auto &state = data_p.global_state->Cast<AbsolutePositionGlobalState>();
    
    const auto &bvh = *bind_data.bvh_data;
    
    duckdb::idx_t total_rows = bvh.num_frames * bvh.joint_list.size();
    
    if (state.current_row >= total_rows) {
        output.SetCardinality(0);
        return;
    }
    
    AbsolutePositionCalculator calculator(bvh);
    
    duckdb::idx_t count = 0;
    duckdb::idx_t max_rows = STANDARD_VECTOR_SIZE;
    
    auto frame_id_data = duckdb::FlatVector::GetData<int32_t>(output.data[0]);
    auto time_data = duckdb::FlatVector::GetData<double>(output.data[1]);
    auto joint_name_data = duckdb::FlatVector::GetData<duckdb::string_t>(output.data[2]);
    auto world_x_data = duckdb::FlatVector::GetData<double>(output.data[3]);
    auto world_y_data = duckdb::FlatVector::GetData<double>(output.data[4]);
    auto world_z_data = duckdb::FlatVector::GetData<double>(output.data[5]);
    auto rot_x_data = duckdb::FlatVector::GetData<double>(output.data[6]);
    auto rot_y_data = duckdb::FlatVector::GetData<double>(output.data[7]);
    auto rot_z_data = duckdb::FlatVector::GetData<double>(output.data[8]);
    
    while (count < max_rows && state.current_row < total_rows) {
        int frame_id = state.current_row / bvh.joint_list.size();
        int joint_idx = state.current_row % bvh.joint_list.size();
        
        // このフレームの絶対位置を計算
        auto transforms = calculator.calculateFrame(frame_id);
        
        const Joint* joint = bvh.joint_list[joint_idx];
        const auto& transform = transforms[joint->name];
        
        frame_id_data[count] = frame_id;
        time_data[count] = frame_id * bvh.frame_time;
        joint_name_data[count] = duckdb::StringVector::AddString(output.data[2], joint->name);
        world_x_data[count] = transform.world_x;
        world_y_data[count] = transform.world_y;
        world_z_data[count] = transform.world_z;
        rot_x_data[count] = transform.rot_x;
        rot_y_data[count] = transform.rot_y;
        rot_z_data[count] = transform.rot_z;
        
        count++;
        state.current_row++;
    }
    
    output.SetCardinality(count);
}

} // namespace bvh_absolute

// =============================================================================
// Extension Class Implementation
// =============================================================================

namespace duckdb {

void Bvh2sqlExtension::Load(ExtensionLoader &loader) {
    // bvh_absolute_positions関数を登録
    loader.RegisterFunction(TableFunction(
        "bvh_absolute_positions", 
        {LogicalType::VARCHAR}, 
        bvh_absolute::AbsolutePositionFunction, 
        bvh_absolute::AbsolutePositionBind, 
        bvh_absolute::AbsolutePositionInit
    ));
}

std::string Bvh2sqlExtension::Name() {
    return "bvh2sql";
}

std::string Bvh2sqlExtension::Version() const {
#ifdef EXT_VERSION_BVH2SQL
    return EXT_VERSION_BVH2SQL;
#else
    return "1.0.0";
#endif
}

} // namespace duckdb

// =============================================================================
// C Extension Entry Points
// =============================================================================

extern "C" {

DUCKDB_EXTENSION_API void bvh2sql_duckdb_cpp_init(duckdb::ExtensionLoader &loader) {
    duckdb::Bvh2sqlExtension ext;
    ext.Load(loader);
}

DUCKDB_EXTENSION_API void bvh2sql_init(duckdb::DatabaseInstance &db) {
    duckdb::Connection con(db);
    con.BeginTransaction();
    
    auto &catalog = duckdb::Catalog::GetSystemCatalog(*con.context);
    
    // bvh_absolute_positions関数を登録
    duckdb::TableFunction absolute_func(
        "bvh_absolute_positions", 
        {duckdb::LogicalType::VARCHAR}, 
        bvh_absolute::AbsolutePositionFunction, 
        bvh_absolute::AbsolutePositionBind, 
        bvh_absolute::AbsolutePositionInit
    );
    duckdb::CreateTableFunctionInfo absolute_info(absolute_func);
    catalog.CreateTableFunction(*con.context, absolute_info);
    
    con.Commit();
}

DUCKDB_EXTENSION_API const char *bvh2sql_version() {
    return duckdb::DuckDB::LibraryVersion();
}

}