package main

import (
	"net/http"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/o0olele/detour-go/debugger"
)

func main() {
	r := gin.Default()
	r.Use(cors.Default())
	r.StaticFS("/public", http.Dir("./public"))

	server := debugger.NewServer(r)
	_ = server

	r.Run(":9001")
}
