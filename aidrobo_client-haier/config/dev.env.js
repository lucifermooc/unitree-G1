'use strict'
const merge = require('webpack-merge')
const prodEnv = require('./prod.env')
const baseVersion = require('../package.json').version

module.exports = merge(prodEnv, {
  NODE_ENV: '"development"',
  VUE_APP_VERSION: JSON.stringify(baseVersion + '.dev')
})
