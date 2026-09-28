'use strict'
const baseVersion = require('../package.json').version
const buildNumber = process.env.BUILD_NUMBER
const version = buildNumber ? `${baseVersion}.b${buildNumber}` : baseVersion

module.exports = {
  NODE_ENV: '"production"',
  VUE_APP_VERSION: JSON.stringify(version)
}
