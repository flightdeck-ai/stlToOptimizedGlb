#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <thread>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

struct Vertex {
    std::array<float, 3> position;
    std::array<float, 3> color;
};

enum class Output {
    Standard,
    Binary,
};

std::string to_base64(const std::vector<uint8_t>& data) {
    static constexpr char base64_chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string result;
    int i = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];

    for (const unsigned char c : data) {
        char_array_3[i++] = c;
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; i < 4; i++)
                result += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        int j = 0;
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (j = 0; j < i + 1; j++)
            result += base64_chars[char_array_4[j]];

        while(i++ < 3)
            result += '=';
    }

    return result;
}

std::vector<uint8_t> convert_stl_to_glb(const std::vector<uint8_t>& stl_data) {
    // TODO: Assimp doesnt properly support gltf/glb exports. switch library.
    if (stl_data.empty()) {
        throw std::runtime_error("Empty STL data provided");
    }
    Assimp::Importer importer;
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
        aiComponent_COLORS | aiComponent_TEXTURES | aiComponent_NORMALS);

    const aiScene* scene = importer.ReadFileFromMemory(
        stl_data.data(),
        stl_data.size(),
        aiProcess_Triangulate |
        aiProcess_GenNormals |
        aiProcess_ValidateDataStructure |
        aiProcess_OptimizeMeshes |
        aiProcess_JoinIdenticalVertices,
        "stl"
    );

    if (!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
        throw std::runtime_error("Failed to load STL file: " + std::string(importer.GetErrorString()));
    }

    if (!scene->HasMeshes() || scene->mNumMeshes == 0) {
        throw std::runtime_error("STL file contains no valid meshes");
    }

    Assimp::Exporter exporter;

    const aiExportDataBlob *blob = exporter.ExportToBlob(scene, "glb", aiProcess_PreTransformVertices);

    if (!blob || !blob->data || blob->size == 0) {
        throw std::runtime_error("Failed to export to GLB or exported data is empty");
    }

    std::vector<uint8_t> glb_data(static_cast<const uint8_t*>(blob->data),
                                     static_cast<const uint8_t*>(blob->data) + blob->size);

    // Validate GLB header (basic check)
    if (glb_data.size() < 12 ||
        glb_data[0] != 'g' || glb_data[1] != 'l' || glb_data[2] != 'T' || glb_data[3] != 'F') {
        throw std::runtime_error("Generated GLB file has invalid header");
    }

    std::cout << "GLB size: " << glb_data.size() << std::endl;

    return glb_data;
}

std::vector<uint8_t> optimize_stl(const std::vector<uint8_t>& stl_data) {
    Assimp::Importer importer;
    Assimp::Exporter exporter;

    // TODO: Implement properly

    const aiScene* scene = importer.ReadFileFromMemory(
        stl_data.data(),
        stl_data.size(),
        aiProcess_Triangulate |
        aiProcess_JoinIdenticalVertices |
        aiProcess_GenNormals |
        aiProcess_FixInfacingNormals |
        aiProcess_RemoveRedundantMaterials |
        aiProcess_OptimizeMeshes |
        aiProcess_OptimizeGraph
    );

    if (!scene) {
        throw std::runtime_error("Failed to load STL file: " + std::string(importer.GetErrorString()));
    }

    std::vector<uint8_t> optimized_data;
    const aiExportDataBlob *blob = exporter.ExportToBlob(scene, "stl");

    if (!blob) {
        throw std::runtime_error("Failed to export optimized STL");
    }

    optimized_data.resize(blob->size);
    std::memcpy(optimized_data.data(), blob->data, blob->size);

    delete blob;

    return optimized_data;
}

http::message_generator handle_request(http::request<http::string_body> const& req) {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, "Beast");
    res.keep_alive(req.keep_alive());

    std::cout << "Processing request for: " << req.target() << std::endl;
    std::cout << "Method: " << req.method_string() << std::endl;
    std::cout << "Content-Length: " << req.payload_size().value() << std::endl;

    try {
        if (req.method() == http::verb::post) {
            if (req.payload_size().get_value_or(0) == 0) {
                res.result(http::status::bad_request);
                res.set(http::field::content_type, "text/plain");
                res.body() = "Empty request body";
                res.prepare_payload();
                return res;
            }

            const std::vector<uint8_t> input_data(req.body().begin(), req.body().end());

            if (req.target() == "/convert") {
                const auto glb_data = convert_stl_to_glb(input_data);
                res.set(http::field::content_type, "model/gltf-binary");
                res.body() = to_base64(glb_data);
            }
            else if (req.target() == "/optimize") {
                const auto optimized_data = optimize_stl(input_data);
                res.set(http::field::content_type, "model/stl");
                res.body() = to_base64(optimized_data);
            }
            else if (req.target() == "/optimizeConvert") {
                const auto optimized_data = optimize_stl(input_data);
                const auto glb_data = convert_stl_to_glb(optimized_data);
                res.set(http::field::content_type, "model/gltf-binary");
                res.body() = to_base64(glb_data);
            }
            else {
                res.result(http::status::not_found);
                res.set(http::field::content_type, "text/plain");
                res.body() = "Route not found";
            }
        } else {
            res.result(http::status::method_not_allowed);
            res.set(http::field::content_type, "text/plain");
            res.body() = "Method not allowed";
        }
    }
    catch (const std::exception& e) {
        res.result(http::status::internal_server_error);
        res.set(http::field::content_type, "text/plain");
        res.body() = std::string("Error processing request: ") + e.what();
        std::cerr << "Error processing request: " << e.what() << std::endl;
    }

    res.prepare_payload();
    return res;
}

void fail(const beast::error_code &ec, char const* what) {
    std::cerr << what << ": " << ec.message() << "\n";
}

class Session : public std::enable_shared_from_this<Session> {
    beast::tcp_stream stream_;
    beast::flat_buffer buffer_{8192};
    boost::optional<http::request_parser<http::string_body>> parser_;
    http::response<http::string_body> res_;

    static constexpr std::size_t queue_limit = 8; // max responses
    std::queue<http::message_generator> response_queue_;

public:
    explicit Session(tcp::socket&& socket): stream_(std::move(socket))
    {
        static_assert(queue_limit > 0, "queue limit must be positive");
    }

    void run() {
        net::dispatch(
            stream_.get_executor(),
            beast::bind_front_handler(
                &Session::do_read,
                this->shared_from_this()));
    }

private:
    void do_read() {
        parser_.emplace();

        parser_->body_limit((std::numeric_limits<std::uint64_t>::max)());

        stream_.expires_after(std::chrono::seconds(60));

        http::async_read(
            stream_,
            buffer_,
            *parser_,
            beast::bind_front_handler(
                &Session::on_read,
                shared_from_this()));
    }

    void on_read(const beast::error_code &ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if(ec == http::error::end_of_stream)
            return do_close();

        if(ec)
            return fail(ec, "read");

        queue_write(handle_request(parser_->release()));

        if (response_queue_.size() < queue_limit)
            do_read();
    }

    void queue_write(http::message_generator response) {
        response_queue_.push(std::move(response));

        if (response_queue_.size() == 1)
            do_write();
    }

    void do_write() {
        if(!response_queue_.empty()){
            bool keep_alive = response_queue_.front().keep_alive();

            beast::async_write(
                stream_,
                std::move(response_queue_.front()),
                beast::bind_front_handler(
                    &Session::on_write,
                    shared_from_this(),
                    keep_alive));
        }
    }

    void
    on_write(
        const bool keep_alive,
        const beast::error_code &ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if(ec)
            return fail(ec, "write");

        if(!keep_alive)
        {
            return do_close();
        }

        if(response_queue_.size() == queue_limit)
            do_read();

        response_queue_.pop();

        do_write();
    }

    void do_close() {
        beast::error_code ec;
        stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
    }
};

class Listener : public std::enable_shared_from_this<Listener> {
    net::io_context& ioc_;
    tcp::acceptor acceptor_;

public:
    Listener(net::io_context& ioc,
        const tcp::endpoint& endpoint) : ioc_(ioc), acceptor_(net::make_strand(ioc))
    {
        beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if(ec)
        {
            fail(ec, "open");
            return;
        }

        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if(ec)
        {
            fail(ec, "set_option");
            return;
        }

        acceptor_.bind(endpoint, ec);
        if(ec)
        {
            fail(ec, "bind");
            return;
        }

        acceptor_.listen(
            net::socket_base::max_listen_connections, ec);
        if(ec)
        {
            fail(ec, "listen");
            return;
        }
    }

    void run()
    {
        net::dispatch(
            acceptor_.get_executor(),
            beast::bind_front_handler(
                &Listener::do_accept,
                this->shared_from_this()));
    }


private:
    void do_accept() {
        acceptor_.async_accept(
            net::make_strand(ioc_),
            beast::bind_front_handler(
                &Listener::on_accept,
                shared_from_this()));
    }

    void on_accept(const beast::error_code &ec, tcp::socket socket) {
        if(ec)
        {
            fail(ec, "accept");
        }
        else
        {
            std::make_shared<Session>(std::move(socket))->run();
        }

        do_accept();
    }

};

int main() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        constexpr unsigned short port = 8080;
        constexpr auto threads = 1;

        net::io_context ioc{threads};

        std::make_shared<Listener>(ioc, tcp::endpoint{address, port})->run();
        std::cout << "listening on " << address << ":" << port << std::endl;

        // std::vector<std::thread> v;
        // v.reserve(threads - 1);
        // for(auto i = threads - 1; i > 0; --i)
        //     v.emplace_back(
        //     [&ioc]
        //     {
        //         ioc.run();
        //     });
        ioc.run();


        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}