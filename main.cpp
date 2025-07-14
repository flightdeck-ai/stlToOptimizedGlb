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

std::vector<uint8_t> convert_stl_to_glb(const std::vector<uint8_t>& stl_data) {
    if (stl_data.empty()) {
        throw std::runtime_error("Empty STL data provided");
    }
    Assimp::Importer importer;

    // Load STL
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

    // Compute centroid
    aiVector3D centroid(0.0f, 0.0f, 0.0f);
    size_t vertexCount = 0;
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
        const aiMesh* mesh = scene->mMeshes[i];
        for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
            centroid += mesh->mVertices[j];
            vertexCount++;
        }
    }
    if (vertexCount == 0) throw std::runtime_error("No vertices found");
    centroid /= static_cast<float>(vertexCount);

    // Create a mutable copy of the scene
    aiScene* modifiedScene = importer.GetOrphanedScene();
    aiVector3D translation(-centroid.x, -centroid.y, -centroid.z);

    // Translate all vertices to center at origin
    for (unsigned int i = 0; i < modifiedScene->mNumMeshes; ++i) {
        aiMesh* mesh = modifiedScene->mMeshes[i];
        for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
            mesh->mVertices[j] += translation;
        }
    }

    // Export to GLB
    Assimp::Exporter exporter;
    const aiExportDataBlob* blob = exporter.ExportToBlob(
        modifiedScene,
        "glb2",
        aiProcess_PreTransformVertices
    );

    if (!blob || !blob->data || blob->size == 0) {
        throw std::runtime_error("Failed to export GLB");
    }

    std::vector<uint8_t> glb_data(
        static_cast<const uint8_t*>(blob->data),
        static_cast<const uint8_t*>(blob->data) + blob->size
    );

    // Validate GLB header
    if (glb_data.size() < 12 || !std::equal(glb_data.begin(), glb_data.begin()+4, "glTF")) {
        throw std::runtime_error("Invalid GLB header");
    }

    return glb_data;
}

std::vector<uint8_t> optimize_stl(const std::vector<uint8_t>& stl_data) {
    Assimp::Importer importer;
    Assimp::Exporter exporter;

    // TODO: Implement properly (Maybe)

    const aiScene* scene = importer.ReadFileFromMemory(
        stl_data.data(),
        stl_data.size(),
        aiProcess_Triangulate |
        aiProcess_JoinIdenticalVertices |
        aiProcess_GenNormals |
        aiProcess_FixInfacingNormals |
        aiProcess_RemoveRedundantMaterials |
        aiProcess_OptimizeMeshes |
        aiProcess_OptimizeGraph,
        "stl"
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

    return optimized_data;
}

http::message_generator handle_request(http::request<http::string_body> const& req) {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, "Beast");
    res.keep_alive(req.keep_alive());

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
                res.body() = std::string(glb_data.begin(), glb_data.end());
            }
            else if (req.target() == "/optimize") {
                const auto optimized_data = optimize_stl(input_data);
                res.set(http::field::content_type, "model/stl");
                res.body() = std::string(optimized_data.begin(), optimized_data.end());
            }
            else if (req.target() == "/optimizeConvert") {
                const auto optimized_data = optimize_stl(input_data);
                const auto glb_data = convert_stl_to_glb(optimized_data);
                res.set(http::field::content_type, "model/gltf-binary");
                res.body() = std::string(glb_data.begin(), glb_data.end());
            }
            else {
                res.result(http::status::not_found);
                res.set(http::field::content_type, "text/plain");
                res.body() = "Route not found";
            }
        } else {
            if (req.method() == http::verb::get && req.target() == "/health") {
                res.result(http::status::ok);
                res.set(http::field::content_type, "text/plain");
                res.body() = "OK";
            } else {
                res.result(http::status::method_not_allowed);
                res.set(http::field::content_type, "text/plain");
                res.body() = "Method not allowed";
            }
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

            beast::async_write(
                stream_,
                std::move(response_queue_.front()),
                beast::bind_front_handler(
                    &Session::on_write,
                    shared_from_this(),
                    false));
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

        // UNCOMMENT THIS IF MT is used
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