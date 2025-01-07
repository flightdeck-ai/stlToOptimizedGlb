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

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

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
    Assimp::Importer importer;
    Assimp::Exporter exporter;

    std::cout << "Converting stl data..." << std::endl;

    const aiScene* scene = importer.ReadFileFromMemory(
        stl_data.data(),
        stl_data.size(),
        aiProcess_Triangulate | aiProcess_GenNormals
    );

    if (!scene) {
        throw std::runtime_error("Failed to load STL file: " + std::string(importer.GetErrorString()));
    }

    std::vector<uint8_t> glb_data;
    const aiExportDataBlob *blob = exporter.ExportToBlob(scene, "glb");

    if (!blob) {
        throw std::runtime_error("Failed to export to GLB");
    }

    glb_data.resize(blob->size);
    std::memcpy(glb_data.data(), blob->data, blob->size);

    delete blob;

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

http::response<http::string_body> handle_request(http::request<http::string_body> const& req) {
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

            std::vector<uint8_t> input_data(req.body().begin(), req.body().end());

            if (req.target() == "/convert") {
                auto glb_data = convert_stl_to_glb(input_data);
                res.set(http::field::content_type, "model/gltf-binary");
                res.body() = to_base64(glb_data);
            }
            else if (req.target() == "/optimize") {
                auto optimized_data = optimize_stl(input_data);
                res.set(http::field::content_type, "model/stl");
                res.body() = to_base64(optimized_data);
            }
            else if (req.target() == "/optimizeConvert") {
                auto optimized_data = optimize_stl(input_data);
                auto glb_data = convert_stl_to_glb(optimized_data);
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

class Session : public std::enable_shared_from_this<Session> {
    tcp::socket socket_;
    beast::flat_buffer buffer_{8192};
    http::request<http::string_body> req_;
    http::response<http::string_body> res_;

public:
    explicit Session(tcp::socket socket) : socket_(std::move(socket)) {}

    void run() {
        do_read();
    }

private:
    void do_read() {
        auto self(shared_from_this());

        req_ = {};

        http::response_parser<http::file_body> parser;
        parser.body_limit((std::numeric_limits<std::uint64_t>::max)());
        // TODO: Figure this out

        http::async_read(
            socket_,
            buffer_,
            req_,
            [self](beast::error_code ec, std::size_t bytes_transferred) {
                if (ec == http::error::end_of_stream) {
                    return self->do_close();
                }

                if (ec) {
                    std::cerr << "Read error: " << ec.message() << std::endl;
                    return;
                }

                std::cout << "Received request: " << self->req_.method_string() << " "
                         << self->req_.target() << std::endl;
                std::cout << "Content length: " << self->req_.body().length() << std::endl;

                self->res_ = handle_request(self->req_);
                self->do_write();
            });
    }

    void do_write() {
        auto self(shared_from_this());

        res_.prepare_payload();

        http::async_write(
            socket_,
            res_,
            [self](beast::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Write error: " << ec.message() << std::endl;
                    return;
                }

                if (self->req_.need_eof()) {
                    return self->do_close();
                }
                self->do_read();
            });
    }

    void do_close() {
        beast::error_code ec;
        socket_.shutdown(tcp::socket::shutdown_send, ec);
    }
};

class Listener : public std::enable_shared_from_this<Listener> {
    net::io_context& ioc_;
    tcp::acceptor acceptor_;

public:
    Listener(net::io_context& ioc, const tcp::endpoint& endpoint)
        : ioc_(ioc), acceptor_(net::make_strand(ioc)) {
        beast::error_code ec;

        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            std::cerr << "Open error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec) {
            std::cerr << "Set option error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.bind(endpoint, ec);
        if (ec) {
            std::cerr << "Bind error: " << ec.message() << std::endl;
            return;
        }

        acceptor_.listen(net::socket_base::max_listen_connections, ec);
        if (ec) {
            std::cerr << "Listen error: " << ec.message() << std::endl;
            return;
        }

        do_accept();
    }

private:
    void do_accept() {
        acceptor_.async_accept(net::make_strand(ioc_), [this](const beast::error_code &ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<Session>(std::move(socket))->run();
            }
            do_accept();
        });
    }
};

int main() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        constexpr unsigned short port = 8080;

        net::io_context ioc{1};

        auto listener = std::make_shared<Listener>(ioc, tcp::endpoint{address, port});

        ioc.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}